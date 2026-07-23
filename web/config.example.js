// Copy to `config.js` (untracked) and set your own broker and topics.
// Do NOT put AES keys here — enter them in the app's key vault (🗝) so they
// stay in tab memory only and are wiped when the page closes.
window.ZENITH_CONFIG = {
  broker: "wss://broker.example.com:8884/mqtt",
  devices: [
    { id: "trk01", name: "Vehicle 1",
      topic: "yourprefix/trk01/pos",
      cmd:   "yourprefix/trk01/cmd" }
  ]
};
