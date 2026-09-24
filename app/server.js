const express = require("express");
const mongoose = require("mongoose");
const cors = require("cors");

const http = require("http");
const path = require("path");

const WebSocket = require("ws");
const { WebSocketServer } = WebSocket;

const userRoutes = require("./routes/users");


const app = express();


// ============================================================
// MIDDLEWARE
// ============================================================

app.use(cors());

app.use(express.json());


// ============================================================
// PUBLIC WEB APPLICATION
// ============================================================

app.use(
  express.static(
    path.join(__dirname, "public")
  )
);


// ============================================================
// MONGODB
// ============================================================

mongoose
  .connect(
    "mongodb://127.0.0.1:27017/simple_crud"
  )
  .then(() => {

    console.log(
      "MongoDB connected"
    );

  })
  .catch((error) => {

    console.error(
      "MongoDB connection error:",
      error
    );

  });


// ============================================================
// HTTP SERVER
//
// We use http.createServer instead of app.listen because
// WebSocket uses the same server/port.
// ============================================================

const server = http.createServer(app);


// ============================================================
// WEBSOCKET SERVER
// ============================================================

const wss = new WebSocketServer({
  server,
  path: "/ws"
});


// ============================================================
// BROADCAST FUNCTION
//
// routes/users.js can call:
//
// req.app.locals.broadcastAccessEvent(...)
//
// Browser clients receive the event immediately.
// ============================================================

function broadcastAccessEvent(data) {

  const message = JSON.stringify(data);

  let sentCount = 0;


  wss.clients.forEach((client) => {

    if (
      client.readyState === WebSocket.OPEN
    ) {

      client.send(message);

      sentCount++;
    }

  });


  console.log(
    `WebSocket event broadcast to ${sentCount} client(s)`
  );
}


app.locals.broadcastAccessEvent =
  broadcastAccessEvent;


// ============================================================
// WEBSOCKET CONNECTION
// ============================================================

wss.on("connection", (socket, request) => {

  console.log(
    "Desk UI connected via WebSocket"
  );


  // Initial message
  socket.send(
    JSON.stringify({
      type: "connection",
      connected: true,
      message: "Connected to Fastlane access server",
      timestamp: new Date().toISOString()
    })
  );


  socket.on("close", () => {

    console.log(
      "Desk UI disconnected"
    );

  });


  socket.on("error", (error) => {

    console.error(
      "WebSocket client error:",
      error.message
    );

  });

});


// ============================================================
// API ROUTES
// ============================================================

app.use(
  "/api/users",
  userRoutes
);


// ============================================================
// API TEST
// ============================================================

app.get("/api", (req, res) => {

  res.json({
    success: true,
    message:
      "Fastlane RFID Access Control API",
    websocket: "/ws"
  });

});


// ============================================================
// FRONTEND FALLBACK
// ============================================================

app.get("/", (req, res) => {

  res.sendFile(
    path.join(
      __dirname,
      "public",
      "index.html"
    )
  );

});


// ============================================================
// SERVER
// ============================================================

const PORT = 3000;


server.listen(
  PORT,
  "0.0.0.0",
  () => {

    console.log("");
    console.log(
      "========================================"
    );

    console.log(
      "FASTLANE RFID ACCESS SERVER"
    );

    console.log(
      "========================================"
    );

    console.log(
      `HTTP / UI: http://localhost:${PORT}`
    );

    console.log(
      `WebSocket: ws://localhost:${PORT}/ws`
    );

    console.log(
      `ESP32 API: http://<PC-IP>:${PORT}/api/users/login/:rfid`
    );

    console.log("");

  }
);