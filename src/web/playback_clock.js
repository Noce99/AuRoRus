"use strict";

// ---------------------------------------------------------------------
// PlaybackClock - the playback time of every replaying UI in this project
// (`replay_web_gui` replaying a `.debug` recording, `benchmark_viewer`
// replaying benchmarks): what time it is, whether it's playing, and how
// fast, with listeners told on every change. Served at /playback_clock.js
// by both binaries, after /map_view.js (it asks `MapView` to redraw).
// ---------------------------------------------------------------------

window.PlaybackClock = {
  durationUs: 0,
  currentTimeUs: 0,
  playing: false,
  speedMultiplier: 1,
  /** @type {((timeUs:number) => void)[]} */
  listeners: [],
  _lastFrameMs: null,

  subscribe(fn) {
    this.listeners.push(fn);
  },

  _notify() {
    for (const fn of this.listeners) fn(this.currentTimeUs);
    MapView.requestRedraw();
  },

  setTime(timeUs) {
    this.currentTimeUs = Math.max(0, Math.min(this.durationUs, timeUs));
    this._notify();
  },

  play() {
    if (this.currentTimeUs >= this.durationUs) this.currentTimeUs = 0;
    this.playing = true;
    this._lastFrameMs = null;
    requestAnimationFrame((ms) => this._tick(ms));
  },

  pause() {
    this.playing = false;
  },

  togglePlaying() {
    if (this.playing) this.pause();
    else this.play();
  },

  _tick(nowMs) {
    if (!this.playing) return;
    if (this._lastFrameMs !== null) {
      const deltaUs = (nowMs - this._lastFrameMs) * 1000 * this.speedMultiplier;
      this.currentTimeUs = Math.min(this.durationUs, this.currentTimeUs + deltaUs);
    }
    this._lastFrameMs = nowMs;
    this._notify();
    if (this.currentTimeUs >= this.durationUs) {
      this.playing = false;
      return;
    }
    requestAnimationFrame((ms) => this._tick(ms));
  },
};
