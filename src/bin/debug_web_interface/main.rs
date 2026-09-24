//! `debug_web_interface`: serves a read-only playback UI for a `.debug`
//! session file recorded by `web_gui --debug` - the same map/vehicle canvas
//! as `web_gui`, plus a bottom timeline panel. Entirely self-contained from
//! the recorded file: no `Runner`/`Captain`/`Executor`, no dependency on the
//! `config/`/`maps/` folders the session was originally recorded against.

mod assets;
mod cli;
mod debug_api;
mod handlers;
mod session;

use session::Session;

fn main() {
    let config = cli::parse_config(std::env::args());

    let session = Session::load(&config.file).unwrap_or_else(|err| {
        eprintln!("debug_web_interface: failed to read {:?}: {err}", config.file);
        std::process::exit(1);
    });

    let server = aurorus::web::bind_http(config.bind_addr.as_str()).unwrap_or_else(|err| {
        eprintln!("debug_web_interface: failed to bind {}: {err}", config.bind_addr);
        std::process::exit(1);
    });
    println!("debug_web_interface: serving {:?} on http://{}", config.file, config.bind_addr);

    for request in server.incoming_requests() {
        handlers::handle(request, &session);
    }
}
