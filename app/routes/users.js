const express = require("express");
const User = require("../models/User");

const router = express.Router();


// ============================================================
// RFID NORMALIZATION
// ============================================================

function normalizeRFID(rfid) {

  if (!rfid) {
    return null;
  }


  return rfid
    .toString()
    .trim()
    .toUpperCase()
    .replace(/-/g, ":");
}


// ============================================================
// WEBSOCKET BROADCAST HELPER
// ============================================================

function broadcastAccess(
  req,
  event
) {

  try {

    const broadcast =
      req.app.locals.broadcastAccessEvent;


    if (
      typeof broadcast === "function"
    ) {

      broadcast(event);

    }

  } catch (error) {

    // WebSocket notification must NEVER break
    // the ESP32 HTTP API.

    console.error(
      "WebSocket broadcast error:",
      error.message
    );

  }
}


// ============================================================
// CREATE USER
//
// POST /api/users
// ============================================================

router.post("/", async (req, res) => {

  try {

    const rfid = normalizeRFID(
      req.body.rfid
    );


    if (!rfid) {

      return res.status(400).json({
        success: false,
        message: "RFID is required"
      });

    }


    const existingRFID =
      await User.findOne({
        rfid
      });


    if (existingRFID) {

      return res.status(409).json({
        success: false,
        message:
          "RFID card is already registered"
      });

    }


    const user = await User.create({

      name: req.body.name,

      email: req.body.email,

      age: req.body.age,

      department:
        req.body.department || "",

      profileImage:
        req.body.profileImage || "",

      rfid,

      isActive:
        req.body.isActive !== undefined
          ? req.body.isActive
          : true

    });


    return res.status(201).json({
      success: true,
      message:
        "User registered successfully",
      data: user
    });


  } catch (error) {

    if (error.code === 11000) {

      return res.status(409).json({
        success: false,
        message:
          "Email or RFID is already registered"
      });

    }


    return res.status(400).json({
      success: false,
      message: error.message
    });

  }

});


// ============================================================
// RFID ACCESS LOGIN
//
// ESP32:
//
// GET /api/users/login/8A-C1-B4-1A
//
// Backend:
//
// converts to:
//
// 8A:C1:B4:1A
// ============================================================

router.get(
  "/login/:rfid",
  async (req, res) => {

    const requestedAt =
      new Date().toISOString();


    try {

      const rfid = normalizeRFID(
        req.params.rfid
      );


      // ======================================================
      // INVALID REQUEST
      // ======================================================

      if (!rfid) {

        const response = {
          success: false,
          accessGranted: false,
          message: "RFID is required"
        };


        res.status(400).json(
          response
        );


        broadcastAccess(
          req,
          {
            type: "access_attempt",

            status: "denied",

            accessGranted: false,

            reason:
              "RFID is required",

            message:
              "RFID is required",

            rfid: null,

            user: null,

            timestamp: requestedAt
          }
        );


        return;

      }


      // ======================================================
      // FIND USER
      // ======================================================

      const user = await User.findOne({
        rfid
      });


      // ======================================================
      // UNKNOWN RFID
      // ======================================================

      if (!user) {

        const response = {
          success: false,

          accessGranted: false,

          message:
            "RFID card not registered",

          rfid
        };


        res.status(404).json(
          response
        );


        broadcastAccess(
          req,
          {
            type: "access_attempt",

            status: "denied",

            accessGranted: false,

            reason:
              "RFID card not registered",

            message:
              "RFID card not registered",

            rfid,

            user: null,

            timestamp: requestedAt
          }
        );


        return;

      }


      // ======================================================
      // USER DISABLED
      // ======================================================

      if (!user.isActive) {

        const response = {

          success: true,

          accessGranted: false,

          message:
            "User access is disabled",

          rfid,

          user: {
            id: user._id,
            name: user.name
          }

        };


        res.status(403).json(
          response
        );


        broadcastAccess(
          req,
          {

            type: "access_attempt",

            status: "denied",

            accessGranted: false,

            reason:
              "User access is disabled",

            message:
              "User access is disabled",

            rfid,

            user: {
              id: user._id,

              name: user.name,

              email: user.email,

              department:
                user.department,

              profileImage:
                user.profileImage,

              rfid: user.rfid
            },

            timestamp: requestedAt

          }
        );


        return;

      }


      // ======================================================
      // ACCESS GRANTED
      // ======================================================

      const response = {

        success: true,

        accessGranted: true,

        message:
          "Access granted",

        user: {

          id: user._id,

          name: user.name,

          email: user.email,

          age: user.age,

          department:
            user.department,

          profileImage:
            user.profileImage,

          rfid: user.rfid

        }

      };


      // ------------------------------------------------------
      // ESP32 gets HTTP response normally.
      // ------------------------------------------------------

      res.status(200).json(
        response
      );


      // ------------------------------------------------------
      // Desk UI gets WebSocket notification.
      // ------------------------------------------------------

      broadcastAccess(
        req,
        {

          type: "access_attempt",

          status: "granted",

          accessGranted: true,

          reason: null,

          message:
            "Access granted",

          rfid: user.rfid,

          user: {

            id: user._id,

            name: user.name,

            email: user.email,

            department:
              user.department,

            profileImage:
              user.profileImage,

            rfid: user.rfid

          },

          timestamp: requestedAt

        }
      );


    } catch (error) {

      console.error(
        "RFID login error:",
        error
      );


      // ======================================================
      // SERVER ERROR
      //
      // IMPORTANT:
      // Do NOT call this a real RFID denial.
      // ======================================================

      res.status(500).json({

        success: false,

        accessGranted: false,

        message:
          "Internal server error"

      });


      broadcastAccess(
        req,
        {

          type: "access_attempt",

          status: "error",

          accessGranted: false,

          reason:
            "Internal server error",

          message:
            "Unable to verify RFID",

          rfid:
            normalizeRFID(
              req.params.rfid
            ),

          user: null,

          timestamp: requestedAt

        }
      );

    }

  }
);


// ============================================================
// READ ALL
//
// GET /api/users
// ============================================================

router.get("/", async (req, res) => {

  try {

    const users =
      await User.find().sort({
        createdAt: -1
      });


    return res.json({
      success: true,
      data: users
    });


  } catch (error) {

    return res.status(500).json({
      success: false,
      message: error.message
    });

  }

});


// ============================================================
// READ ONE
//
// GET /api/users/:id
// ============================================================

router.get("/:id", async (req, res) => {

  try {

    const user =
      await User.findById(
        req.params.id
      );


    if (!user) {

      return res.status(404).json({
        success: false,
        message: "User not found"
      });

    }


    return res.json({
      success: true,
      data: user
    });


  } catch (error) {

    return res.status(400).json({
      success: false,
      message: error.message
    });

  }

});


// ============================================================
// UPDATE
//
// PUT /api/users/:id
// ============================================================

router.put("/:id", async (req, res) => {

  try {

    const updateData = {};


    if (
      req.body.name !== undefined
    ) {
      updateData.name =
        req.body.name;
    }


    if (
      req.body.email !== undefined
    ) {
      updateData.email =
        req.body.email;
    }


    if (
      req.body.age !== undefined
    ) {
      updateData.age =
        req.body.age;
    }


    if (
      req.body.department !== undefined
    ) {
      updateData.department =
        req.body.department;
    }


    if (
      req.body.profileImage !== undefined
    ) {
      updateData.profileImage =
        req.body.profileImage;
    }


    if (
      req.body.isActive !== undefined
    ) {
      updateData.isActive =
        req.body.isActive;
    }


    // ========================================================
    // RFID UPDATE
    // ========================================================

    if (
      req.body.rfid !== undefined
    ) {

      const rfid =
        normalizeRFID(
          req.body.rfid
        );


      if (!rfid) {

        return res.status(400).json({
          success: false,
          message: "Invalid RFID"
        });

      }


      const existingRFID =
        await User.findOne({

          rfid,

          _id: {
            $ne: req.params.id
          }

        });


      if (existingRFID) {

        return res.status(409).json({

          success: false,

          message:
            "RFID card is already registered to another user"

        });

      }


      updateData.rfid = rfid;

    }


    const user =
      await User.findByIdAndUpdate(

        req.params.id,

        updateData,

        {
          new: true,
          runValidators: true
        }

      );


    if (!user) {

      return res.status(404).json({
        success: false,
        message: "User not found"
      });

    }


    return res.json({

      success: true,

      message:
        "User updated successfully",

      data: user

    });


  } catch (error) {

    if (error.code === 11000) {

      return res.status(409).json({

        success: false,

        message:
          "Email or RFID is already registered"

      });

    }


    return res.status(400).json({
      success: false,
      message: error.message
    });

  }

});


// ============================================================
// DELETE
//
// DELETE /api/users/:id
// ============================================================

router.delete("/:id", async (req, res) => {

  try {

    const user =
      await User.findByIdAndDelete(
        req.params.id
      );


    if (!user) {

      return res.status(404).json({
        success: false,
        message: "User not found"
      });

    }


    return res.json({

      success: true,

      message:
        "User deleted"

    });


  } catch (error) {

    return res.status(400).json({
      success: false,
      message: error.message
    });

  }

});


module.exports = router;