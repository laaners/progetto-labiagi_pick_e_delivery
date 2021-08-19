'use strict';

const express = require('express');
const http = require('http');
const cors = require('cors');
const WebSocket = require('ws');

var app = express();
app.use(cors());
const port = 3000;

const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

rosnodejs.initNode('/talker_node', {onTheFly: true}).then((rosNode) => {
    let tosend = {"position": null, "goal": null};

    let sub_pos = rosNode.subscribe('/pick_e_delivery/Pose', 'pick_e_delivery/Pose',
        (data) => { // define callback execution            
            //rosnodejs.log.info('I heard: [' + JSON.stringify(data) + ']');
            tosend.position = data;
            rosnodejs.log.info(JSON.stringify(tosend));
            sendAll(tosend);
        }
    );

    let sub_goal = rosNode.subscribe('/move_base_simple/goal','geometry_msgs/PoseStamped',
        (data) => {
            rosnodejs.log.info('I heard: [' + JSON.stringify({"goal": data}) + ']');
            console.log(JSON.parse(JSON.stringify({"goal": data})));
            tosend.goal = data;
            sendAll(tosend);
        }
    );
    console.log(tosend);
    //sendAll(tosend);

});

app.get('/', function(req,res) {
    res.sendFile("index.html",{root: __dirname});
});

app.get('/pick', function(req,res) {
    rosnodejs.initNode('/talker_node', {onTheFly: true}).then((rosNode) => {
        let pub_goal = rosNode.advertise('/New_Goal','pick_e_delivery/NewGoal', {
            queueSize: 1,
            latching: true,
            throttleMs: 9
        });
        pub_goal.publish({x: 8.0, y: 22.0, theta: 0.0, status: 1});
    });
    res.send("pick");
});

app.get('/delivery', function(req,res) {
    rosnodejs.initNode('/talker_node').then((rosNode) => {
        let pub_goal = rosNode.advertise('/New_Goal','pick_e_delivery/NewGoal', {
            queueSize: 1,
            latching: true,
            throttleMs: 9
        });
        pub_goal.publish({x: 9.0, y: 19.0, theta: 0.0, status: 2});
    });
    res.send("delivery");
});

app.get('/pick_pack', function(req,res) {
    rosnodejs.initNode('/talker_node').then((rosNode) => {
        let pub_goal = rosNode.advertise('/New_Goal','pick_e_delivery/NewGoal', {
            queueSize: 1,
            latching: true,
            throttleMs: 9
        });
        //pub_goal.publish({x: 9.0, y: 19.0, theta: 0.0, status: 0});
        pub_goal.publish({x: -1.0, y: -1.0, theta: 0.0, status: 0});
    });
    res.send("delivery");
});

app.get('/map', function(req,res) {
    res.sendFile("april_tag_scale.png",{root: __dirname});
});

const httpServer = http.createServer(app);
const ws = new WebSocket.Server({server: httpServer});

var CLIENTS = [];

ws.on('connection', function(conn) {
    CLIENTS.push(conn);
    console.log("Connessione aperta");

    conn.on('message', function(message) {
        console.log("Ricevuto: "+message);
        sendAll(message);
    });

    conn.on('close', function() {
        console.log("Connessione chiusa");
        CLIENTS.splice(CLIENTS.indexOf(conn),1);
    });
});

function sendAll(message) {
    for(let i = 0; i < CLIENTS.length; i++) {
        var j = i+1;
        CLIENTS[i].send(JSON.stringify(message));
    }
}

httpServer.listen(port, function() {
    console.log("In ascolto sulla porta: "+port);
});