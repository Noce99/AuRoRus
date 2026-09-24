//! The bits every web UI in this project needs: the handful of `tiny_http`
//! response helpers each one was otherwise re-implementing, and the
//! frontend assets they share.
//!
//! Both `web_gui` (driving live) and `replay_web_gui` (replaying a
//! recording) show the same map canvas, fed the same way - through the
//! drawing protocol in [`draw`] - so the JavaScript that paints it
//! ([`MAP_VIEW_JS`]), the client of that protocol ([`DRAW_LAYERS_JS`]), and
//! the CSS that frames them ([`BASE_CSS`]) live here once and are served by
//! both, ahead of each binary's own `app.js` and `style.css`.

pub mod draw;

use std::error::Error;
use std::io::{self, ErrorKind, Write};
use std::net::{TcpListener, ToSocketAddrs};
use tiny_http::{Method, Request, Response, ResponseBox};

/// The shared map-canvas frontend, served at `/map_view.js`.
pub const MAP_VIEW_JS: &str = include_str!("web/map_view.js");
/// The shared drawing-protocol client, served at `/draw_layers.js`.
pub const DRAW_LAYERS_JS: &str = include_str!("web/draw_layers.js");
/// The shared page/canvas styles, served at `/base.css`.
pub const BASE_CSS: &str = include_str!("web/base.css");

/// Binds a `tiny_http` HTTP server to `addr`, like `tiny_http::Server::http`,
/// but with `TCP_NODELAY` set on every connection it accepts.
///
/// `tiny_http` writes each response through a 1 KiB buffer, so anything
/// larger (e.g. a ~3 KiB LIDAR scan) leaves in several small writes. With
/// Nagle's algorithm on, the last of them waits for the client to ACK the
/// previous one - and browsers delay that ACK by ~40 ms on a keep-alive
/// connection, which capped polling any >1 KiB endpoint at ~23 Hz.
/// `tiny_http` has no option for it, so it's set on the listening socket
/// instead, which Linux (and macOS) accepted sockets inherit. Elsewhere this
/// is a plain bind.
pub fn bind_http(
    addr: impl ToSocketAddrs,
) -> Result<tiny_http::Server, Box<dyn Error + Send + Sync>> {
    let listener = TcpListener::bind(addr)?;
    #[cfg(unix)]
    set_nodelay(&listener)?;
    tiny_http::Server::from_listener(listener, None)
}

#[cfg(unix)]
fn set_nodelay(listener: &TcpListener) -> std::io::Result<()> {
    use std::os::fd::AsRawFd;
    let enable: libc::c_int = 1;
    // SAFETY: `listener` owns a valid, open socket fd for the duration of this
    // call, and `enable` is a live `c_int` whose exact size is passed as optlen.
    let result = unsafe {
        libc::setsockopt(
            listener.as_raw_fd(),
            libc::IPPROTO_TCP,
            libc::TCP_NODELAY,
            (&enable as *const libc::c_int).cast(),
            std::mem::size_of::<libc::c_int>() as libc::socklen_t,
        )
    };
    if result == 0 {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
    }
}

/// Sends `response` to `request` with a `Connection: close` header, so the
/// client opens a fresh connection for its next request instead of reusing
/// this one.
///
/// That matters to a server that can be torn down and rebuilt while clients
/// stay connected - e.g. `web_gui`, which [`crate::Captain::request_restart`]
/// rebuilds from scratch. Dropping a `tiny_http::Server` doesn't close the
/// connections it already accepted: a client that reuses one after the
/// rebuild sends its request into a server nobody reads from anymore, and
/// waits for an answer forever. Closing every connection after one response
/// means no request ever lands on a stale one.
///
/// `tiny_http` refuses a `Connection` header on a response (`add_header`
/// drops it silently) and has no option to close the connection itself, so
/// the response is formatted into a buffer by `tiny_http` as usual and the
/// header spliced in right after the status line. Every response here is
/// built in memory anyway, so the extra buffer costs little. The client
/// closing its end is what ends the connection on this side.
pub fn respond_and_close(request: Request, response: ResponseBox) -> io::Result<()> {
    let http_version = request.http_version().clone();
    let request_headers = request.headers().to_vec();
    let head_only = *request.method() == Method::Head;

    let mut raw = Vec::new();
    response.raw_print(&mut raw, http_version, &request_headers, head_only, None)?;
    let status_line_end = raw
        .windows(2)
        .position(|pair| pair == b"\r\n")
        .map(|i| i + 2)
        .expect("tiny_http always writes a CRLF-terminated status line");
    raw.splice(
        status_line_end..status_line_end,
        b"Connection: close\r\n".iter().copied(),
    );

    let mut writer = request.into_writer();
    // Same as `Request::respond`: a client that hung up before reading its
    // response isn't this server's error.
    writer
        .write_all(&raw)
        .and_then(|()| writer.flush())
        .or_else(|err| match err.kind() {
            ErrorKind::BrokenPipe | ErrorKind::ConnectionAborted | ErrorKind::ConnectionReset => {
                Ok(())
            }
            _ => Err(err),
        })
}

/// Builds a `Content-Type: ...`-style header. A header name/value built
/// from a `&'static str` constant is always valid ASCII, so parsing it can
/// never fail.
pub fn header(name: &str, value: &str) -> tiny_http::Header {
    format!("{name}: {value}")
        .parse()
        .expect("header name/value are always valid ASCII")
}

/// Serves one of the shared assets by request path, or `None` if the path
/// names none of them - let the caller's own router try its own assets
/// next.
pub fn shared_asset(path: &str) -> Option<ResponseBox> {
    let (body, content_type) = match path {
        "/map_view.js" => (MAP_VIEW_JS, "text/javascript; charset=utf-8"),
        "/draw_layers.js" => (DRAW_LAYERS_JS, "text/javascript; charset=utf-8"),
        "/base.css" => (BASE_CSS, "text/css; charset=utf-8"),
        _ => return None,
    };
    Some(
        Response::from_string(body)
            .with_header(header("Content-Type", content_type))
            .boxed(),
    )
}

/// Serializes `value` as a JSON response with the given status code.
pub fn json_response<T: serde::Serialize>(value: &T, status: u16) -> ResponseBox {
    let body =
        serde_json::to_string(value).expect("serializing a well-formed API response never fails");
    Response::from_string(body)
        .with_status_code(status)
        .with_header(header("Content-Type", "application/json"))
        .boxed()
}

#[derive(serde::Serialize)]
struct ErrorBody {
    error: String,
}

/// A JSON `{"error": "..."}` body with the given status code - the shape
/// both frontends' `fetchJSON` knows how to surface.
pub fn error_response(status: u16, message: &str) -> ResponseBox {
    json_response(
        &ErrorBody {
            error: message.to_string(),
        },
        status,
    )
}

pub fn bad_request(message: &str) -> ResponseBox {
    error_response(400, message)
}

pub fn not_found() -> ResponseBox {
    error_response(404, "not found")
}

/// Reads `request`'s body and parses it as JSON, or returns the 400
/// response to send back instead.
pub fn read_json<T: serde::de::DeserializeOwned>(
    request: &mut tiny_http::Request,
) -> Result<T, ResponseBox> {
    let mut body = String::new();
    if let Err(err) = request.as_reader().read_to_string(&mut body) {
        return Err(bad_request(&format!("failed to read request body: {err}")));
    }
    serde_json::from_str(&body).map_err(|err| bad_request(&format!("invalid JSON body: {err}")))
}

/// The percent-decoded value of query parameter `key` in `url` (e.g.
/// `/api/topic?name=draw%2FMapServer`), or `None` if it's absent or isn't
/// valid UTF-8 once decoded.
pub fn query_param(url: &str, key: &str) -> Option<String> {
    let (_, query) = url.split_once('?')?;
    query
        .split('&')
        .filter_map(|pair| pair.split_once('='))
        .find(|(k, _)| percent_decode(k).as_deref() == Some(key))
        .and_then(|(_, value)| percent_decode(value))
}

/// Decodes `%XX` escapes and `+` (a space, in a query string).
fn percent_decode(text: &str) -> Option<String> {
    let bytes = text.as_bytes();
    let mut decoded = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'%' => {
                let hex = std::str::from_utf8(bytes.get(i + 1..i + 3)?).ok()?;
                decoded.push(u8::from_str_radix(hex, 16).ok()?);
                i += 3;
            }
            b'+' => {
                decoded.push(b' ');
                i += 1;
            }
            byte => {
                decoded.push(byte);
                i += 1;
            }
        }
    }
    String::from_utf8(decoded).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::io::Read;
    use std::net::TcpStream;

    /// Sends `raw_request` to a one-shot server that answers it with
    /// `respond_and_close`, and returns everything the server sent back -
    /// read until the connection closed, which is itself part of the check.
    fn round_trip(raw_request: &str) -> String {
        let server = tiny_http::Server::http("127.0.0.1:0").unwrap();
        let addr = server.server_addr().to_ip().unwrap();
        let handle = std::thread::spawn(move || {
            let request = server.recv().unwrap();
            respond_and_close(request, Response::from_string("hello").boxed()).unwrap();
            // Keep the server alive until the client has read everything.
            std::thread::sleep(std::time::Duration::from_millis(100));
        });

        let mut stream = TcpStream::connect(addr).unwrap();
        stream
            .set_read_timeout(Some(std::time::Duration::from_secs(5)))
            .unwrap();
        stream.write_all(raw_request.as_bytes()).unwrap();
        let mut response = String::new();
        let mut buf = [0u8; 1024];
        // Stop once the whole body arrived: the client is the one that closes.
        while !response.ends_with("hello") {
            let n = stream.read(&mut buf).unwrap();
            assert!(n > 0, "server closed before sending the whole response");
            response.push_str(std::str::from_utf8(&buf[..n]).unwrap());
        }
        handle.join().unwrap();
        response
    }

    #[test]
    fn respond_and_close_asks_the_client_to_close_the_connection() {
        let response = round_trip("GET / HTTP/1.1\r\nHost: x\r\n\r\n");

        let (head, body) = response.split_once("\r\n\r\n").unwrap();
        let mut lines = head.split("\r\n");
        assert_eq!(lines.next(), Some("HTTP/1.1 200 OK"));
        assert!(
            lines.any(|line| line.eq_ignore_ascii_case("connection: close")),
            "{head}"
        );
        assert_eq!(body, "hello");
    }

    #[test]
    fn query_param_decodes_percent_escapes() {
        let url = "/api/topic?name=draw%2FMap%20Server&x=1";
        assert_eq!(query_param(url, "name").as_deref(), Some("draw/Map Server"));
        assert_eq!(query_param(url, "x").as_deref(), Some("1"));
        assert_eq!(query_param(url, "missing"), None);
        assert_eq!(query_param("/api/topic", "name"), None);
    }

    #[test]
    fn query_param_rejects_malformed_escapes() {
        assert_eq!(query_param("/a?name=%2", "name"), None);
        assert_eq!(query_param("/a?name=%zz", "name"), None);
    }
}
