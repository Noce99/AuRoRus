//! `replay_web_gui`: serves a read-only playback UI for a `.debug`
//! session file recorded by `web_gui --debug` - the same map canvas as
//! `web_gui`, drawing whatever the recorded executors drew, plus a bottom
//! timeline panel. Entirely self-contained from
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
        eprintln!("replay_web_gui: failed to read {:?}: {err}", config.file);
        std::process::exit(1);
    });

    let server = aurorus::web::bind_http(config.bind_addr.as_str()).unwrap_or_else(|err| {
        eprintln!("replay_web_gui: failed to bind {}: {err}", config.bind_addr);
        std::process::exit(1);
    });
    println!(
        "replay_web_gui: serving {:?} on http://{}",
        config.file, config.bind_addr
    );

    let file_name = config
        .file
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    for request in server.incoming_requests() {
        handlers::handle(request, &session, &file_name);
    }
}
