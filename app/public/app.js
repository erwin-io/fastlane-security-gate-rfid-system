// ============================================================
// FASTLANE DESK STAFF DISPLAY
// WEBSOCKET CLIENT
//
// RFID is intentionally NOT displayed.
// ============================================================


// ============================================================
// DOM
// ============================================================

const idleView =
  document.getElementById(
    "idleView"
  );


const resultView =
  document.getElementById(
    "resultView"
  );


const resultTitle =
  document.getElementById(
    "resultTitle"
  );


const personName =
  document.getElementById(
    "personName"
  );


const personDepartment =
  document.getElementById(
    "personDepartment"
  );


const personEmail =
  document.getElementById(
    "personEmail"
  );


const reasonText =
  document.getElementById(
    "reasonText"
  );


const profileImage =
  document.getElementById(
    "profileImage"
  );


const profileFallback =
  document.getElementById(
    "profileFallback"
  );


const connectionBadge =
  document.getElementById(
    "connectionBadge"
  );


const connectionText =
  document.getElementById(
    "connectionText"
  );


const clock =
  document.getElementById(
    "clock"
  );


// ============================================================
// SETTINGS
// ============================================================

const RESULT_DISPLAY_MS =
  5000;


const WS_RECONNECT_MS =
  2000;


// ============================================================
// STATE
// ============================================================

let socket = null;

let reconnectTimer = null;

let resetTimer = null;


// ============================================================
// CLOCK
// ============================================================

function updateClock() {

  const now =
    new Date();


  clock.textContent =
    now.toLocaleTimeString(
      [],
      {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit"
      }
    );

}


setInterval(
  updateClock,
  1000
);


updateClock();


// ============================================================
// CONNECTION STATUS
// ============================================================

function setConnected(
  connected
) {

  connectionBadge.classList.toggle(
    "online",
    connected
  );


  connectionBadge.classList.toggle(
    "offline",
    !connected
  );


  connectionText.textContent =
    connected
      ? "LIVE"
      : "OFFLINE";

}


// ============================================================
// IDLE
// ============================================================

function activateIdle() {

  clearTimeout(
    resetTimer
  );


  document.body.className =
    "state-idle";


  idleView.classList.add(
    "active"
  );


  resultView.classList.remove(
    "active"
  );

}


// ============================================================
// PROFILE
// ============================================================

function setProfile(
  user
) {

  const image =
    user?.profileImage;


  if (image) {

    profileImage.src =
      image;


    profileImage.classList.remove(
      "hidden"
    );


    profileFallback.classList.add(
      "hidden"
    );

    return;

  }


  profileImage.classList.add(
    "hidden"
  );


  profileFallback.classList.remove(
    "hidden"
  );


  const name =
    user?.name?.trim();


  profileFallback.textContent =
    name
      ? name
          .charAt(0)
          .toUpperCase()
      : "?";

}


// ============================================================
// CLEAR OPTIONAL TEXT
// ============================================================

function clearIdentityFields() {

  personDepartment.textContent =
    "";


  personEmail.textContent =
    "";


  reasonText.textContent =
    "";

}


// ============================================================
// SHOW ACCESS RESULT
// ============================================================

function showAccessResult(
  event
) {

  clearTimeout(
    resetTimer
  );


  clearIdentityFields();


  const user =
    event.user || null;


  // ==========================================================
  // GRANTED
  // ==========================================================

  if (
    event.status === "granted"
  ) {

    document.body.className =
      "state-granted";


    resultTitle.textContent =
      "GRANTED";


    personName.textContent =
      user?.name ||
      "Authorized User";


    if (
      user?.department
    ) {

      personDepartment.textContent =
        user.department;

    }


    if (
      user?.email
    ) {

      personEmail.textContent =
        user.email;

    }


    reasonText.textContent =
      "";


    setProfile(
      user
    );

  }


  // ==========================================================
  // DENIED
  // ==========================================================

  else if (
    event.status === "denied"
  ) {

    document.body.className =
      "state-denied";


    resultTitle.textContent =
      "DENIED";


    // --------------------------------------------------------
    // Known user but access disabled/denied
    // --------------------------------------------------------

    if (user) {

      personName.textContent =
        user.name ||
        "Access Denied";


      if (
        user.department
      ) {

        personDepartment.textContent =
          user.department;

      }


      if (
        user.email
      ) {

        personEmail.textContent =
          user.email;

      }


      setProfile(
        user
      );

    }


    // --------------------------------------------------------
    // Unknown RFID card
    // --------------------------------------------------------

    else {

      personName.textContent =
        "Unknown Card";


      setProfile(
        null
      );

    }


    reasonText.textContent =
      event.reason ||
      event.message ||
      "Access denied";

  }


  // ==========================================================
  // SERVER / CHECK ERROR
  // ==========================================================

  else {

    document.body.className =
      "state-error";


    resultTitle.textContent =
      "CHECK FAILED";


    personName.textContent =
      "Unable to Verify";


    personDepartment.textContent =
      "";


    personEmail.textContent =
      "";


    reasonText.textContent =
      event.reason ||
      event.message ||
      "Unable to verify access";


    setProfile(
      user
    );

  }


  // ==========================================================
  // SHOW RESULT
  // ==========================================================

  idleView.classList.remove(
    "active"
  );


  resultView.classList.add(
    "active"
  );


  // ==========================================================
  // AUTO RETURN TO IDLE
  // ==========================================================

  resetTimer =
    setTimeout(
      activateIdle,
      RESULT_DISPLAY_MS
    );

}


// ============================================================
// WEBSOCKET
// ============================================================

function connectWebSocket() {

  clearTimeout(
    reconnectTimer
  );


  const protocol =
    window.location.protocol ===
    "https:"
      ? "wss:"
      : "ws:";


  const websocketUrl =
    protocol +
    "//" +
    window.location.host +
    "/ws";


  console.log(
    "Connecting WebSocket:",
    websocketUrl
  );


  socket =
    new WebSocket(
      websocketUrl
    );


  // ==========================================================
  // CONNECTED
  // ==========================================================

  socket.addEventListener(
    "open",
    () => {

      console.log(
        "WebSocket connected"
      );


      setConnected(
        true
      );

    }
  );


  // ==========================================================
  // MESSAGE
  // ==========================================================

  socket.addEventListener(
    "message",
    (message) => {

      try {

        const data =
          JSON.parse(
            message.data
          );


        console.log(
          "WebSocket event:",
          data
        );


        // Ignore initial connection message
        if (
          data.type ===
          "connection"
        ) {

          return;

        }


        if (
          data.type ===
          "access_attempt"
        ) {

          showAccessResult(
            data
          );

        }


      } catch (error) {

        console.error(
          "Invalid WebSocket message:",
          error
        );

      }

    }
  );


  // ==========================================================
  // DISCONNECTED
  // ==========================================================

  socket.addEventListener(
    "close",
    () => {

      console.log(
        "WebSocket disconnected"
      );


      setConnected(
        false
      );


      reconnectTimer =
        setTimeout(
          connectWebSocket,
          WS_RECONNECT_MS
        );

    }
  );


  // ==========================================================
  // ERROR
  // ==========================================================

  socket.addEventListener(
    "error",
    (error) => {

      console.error(
        "WebSocket error:",
        error
      );

    }
  );

}


// ============================================================
// START
// ============================================================

activateIdle();

setConnected(
  false
);

connectWebSocket();