// import dependensi
const express = require("express");
const chalk = require("chalk");
const { Server } = require("socket.io");
const http = require("http");
const net = require("net");
const { log } = require("console");
const path = require("path");
const fs = require("fs");

// membuat instans dari dependensi
const app = express();
const server = http.createServer(app);
const io = new Server(server);
const client = new net.Socket();

// variabel essensial
let IP_basestation = "127.0.0.1";
const PORT = 3000;

// middlewares
app.use(express.static("public"));
app.use(express.json());

app.set("view engine", "ejs");
app.set("views", path.join(__dirname, "public"));

// routers
app.get("/", (req, res) => {
  res.sendFile(__dirname + "/public/home.html");
});

app.use((req, res, next) => {
  res.status(404).send("404 | Not found");
});
// routes

// variabel robot-robot
let statusOnline;

let id_robot;

let cmd_receiver = true;

let cmd_status = true;

let cmd_counter = 0;

const sleep = async (ms) => {
  return new Promise((resolve) => {
    setTimeout(resolve, ms);
  });
};

// socket.io events
io.on("connection", (socket) => {
  // event 'ping' untuk cek status online robot
  const ping = () => {
    io.emit("ping", {
      statusOnline,
      IP_basestation,
    });
  };

  const logging = (info, type = "basic") => {
    const logger = {
      basic: chalk,
      good: chalk.greenBright,
      error: chalk.redBright,
      warning: chalk.yellowBright,
    };

    log((logger[type] || chalk)(info));
    io.emit("informasi", {
      info,
      type,
    });
  };

  const goodInfo = (info) => {
    logging(info, "good");
  };

  const warningInfo = (info) => {
    logging(info, "warning");
  };

  const errorInfo = (info) => {
    logging(info, "error");
  };

  // event 'join' saat robot tersambung ke basestation untuk identifikasi
  socket.on("join", (username) => {
    socket.username = username;

    if (!username) return;

    // event 'informasi' untuk kirim informasi terbaru
    goodInfo(username + " terhubung ke Basestation!");

    id_robot = socket.id;
    statusOnline = true;
    cmd_status = true;

    // event 'ping' kirim data robot terbaru
    ping();
  });

  // event 'disconnect' saat ada robot yang terputus koneksi
  socket.on("disconnect", () => {
    if (!socket.username) return;

    warningInfo(socket.username + " offline");

    statusOnline = false;

    // event 'ping' kirim data robot terbaru
    ping();
  });

  socket.on("ping", () => {
    ping();
  });

  const kirimPerintah = (perintah) => {
    if (!statusOnline) {
      errorInfo(`[OFFLINE] ${perintah}`);
      return;
    }

    if (!cmd_receiver) {
      return warningInfo(`Sedang mengirim perintah!`);
    }
    cmd_receiver = false;
    cmd_status = false;

    io.to(id_robot).emit("perintah", perintah);

    logging(`[RUN] ${perintah}`);
  };

  socket.on("cmd_status", ({ msg, isDone }) => {
    if (isDone) logging(`${msg}`);
    else warningInfo(`${msg}`);

    cmd_status = isDone;
  });

  socket.on("received_cmd", () => {
    cmd_receiver = true;
    cmd_counter = 0;
  });

  socket.on("received_perintah", async ({ robot, command }) => {
    await sleep(200);
    if (!cmd_receiver) {
      cmd_counter = cmd_counter + 1;
      if (cmd_counter >= 5) {
        cmd_counter = 0;
        cmd_receiver = true;
        cmd_status = true;
        return errorInfo(`[GAGAL] ${command}`);
      }

      io.to(id_robot).emit("perintah", command);
    }
  });

  //kirim perintah ke robot
  socket.on("kirim-perintah", ({ perintah }) => {
    kirimPerintah(perintah);
    if (!perintah.includes("vision") && perintah != "reset") {
      setTimeout(() => {
        kirimPerintah("berhenti");
      }, 2000);
    }
  });
});

const os = require("os");
const networkInterfaces = os.networkInterfaces();

for (const [name, interfaces] of Object.entries(networkInterfaces)) {
  if (name.toLowerCase().includes("wi-fi")) {
    for (const details of interfaces) {
      if (details.family === "IPv4") {
        IP_basestation = details.address;
        log(chalk.green(`Basestation IP : http://${details.address}:${PORT}`));
      }
    }
  }
}

// mulai server
server.listen(PORT, () => log(`Basestation is running on http://localhost:${PORT} !!`));
