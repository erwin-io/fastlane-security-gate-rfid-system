const mongoose = require("mongoose");

const userSchema = new mongoose.Schema(
  {
    name: {
      type: String,
      required: true,
      trim: true
    },

    email: {
      type: String,
      required: true,
      unique: true,
      trim: true,
      lowercase: true
    },

    age: {
      type: Number
    },

    department: {
      type: String,
      trim: true,
      default: ""
    },

    // Can later contain:
    //
    // /images/user1.jpg
    //
    // or:
    //
    // https://example.com/user1.jpg
    //
    profileImage: {
      type: String,
      trim: true,
      default: ""
    },

    // ========================================================
    // RFID UID
    //
    // Example:
    //
    // 8A:C1:B4:1A
    // ========================================================

    rfid: {
      type: String,
      required: true,
      unique: true,
      trim: true,
      uppercase: true
    },

    isActive: {
      type: Boolean,
      default: true
    }
  },
  {
    timestamps: true
  }
);

module.exports = mongoose.model(
  "User",
  userSchema
);