//! The bits every web UI in this project needs: the handful of `tiny_http`
//! response helpers each one was otherwise re-implementing, and the
//! frontend assets they share.
//!
//! Both `web_gui` (driving live) and `debug_web_interface` (replaying a
//! recording) show the same map canvas, so the JavaScript that draws it
//! ([`MAP_VIEW_JS`]) and the CSS that frames it ([`BASE_CSS`]) live here
//! once and are served by both, ahead of each binary's own `app.js` and
//! `style.css`.

use std::error::Error;
use std::net::{TcpListener, ToSocketAddrs};
use tiny_http::{Response, ResponseBox};

/// The shared map-canvas frontend, served at `/map_view.js`.
pub const MAP_VIEW_JS: &str = include_str!("web/map_view.js");
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
pub fn bind_http(addr: impl ToSocketAddrs) -> Result<tiny_http::Server, Box<dyn Error + Send + Sync>> {
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
    if result == 0 { Ok(()) } else { Err(std::io::Error::last_os_error()) }
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
    let body = serde_json::to_string(value).expect("serializing a well-formed API response never fails");
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
    json_response(&ErrorBody { error: message.to_string() }, status)
}

pub fn bad_request(message: &str) -> ResponseBox {
    error_response(400, message)
}

pub fn not_found() -> ResponseBox {
    error_response(404, "not found")
}
