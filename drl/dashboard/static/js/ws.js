// Websocket client: connects to /ws, auto-reconnects, and forwards parsed
// frames to a handler. Also owns the connection-status indicator in the top bar.

export function setStatus(cls, text) {
  const dot = document.getElementById("status-dot");
  dot.className = "dot " + cls;
  document.getElementById("status-text").textContent = text;
}

export function connect(onFrame) {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onopen = () => setStatus("connected", "connected");
  ws.onclose = () => {
    setStatus("disconnected", "disconnected - retrying");
    setTimeout(() => connect(onFrame), 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let frame;
    try {
      frame = JSON.parse(ev.data);
    } catch (e) {
      return;
    }
    onFrame(frame);
  };
}
