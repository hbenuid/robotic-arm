// The page's heartbeat, in a worker so a hidden tab keeps it (the server stops motion after 2 s of silence).
let token = null;
self.onmessage = (e) => { token = e.data; };
setInterval(() => {
  if (!token) return;
  fetch("/api/cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json", "X-Dashboard-Token": token },
    body: JSON.stringify({ action: "ping" }),
  }).catch(() => {});
}, 250);
