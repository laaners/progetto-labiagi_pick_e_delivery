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
const tf = require('tf-rosnodejs');


function talker() {
    rosnodejs.initNode('/talker_node').then((rosNode) => {
        let pub = rosNode.advertise('New_Goal','pick_e_delivery/NewGoal');
        let count = 0;
        let msg = new std_msgs.String();


      //msg = JSON.parse('{"x": 8.0, "y": 22.0, "theta": 0.0}');
      //console.log(msg);
      //pub.publish(msg);
      
        let published = false;
        setInterval(() => {
            if(!published) {
                msg = JSON.parse('{"x": 8.0, "y": 22.0, "theta": 0.0}');
                //msg = JSON.parse('{"x": 9.0, "y": 7.0, "theta": 0.0}');
                console.log(msg);
                pub.publish(msg);
                published = true;
            }
            // Log through stdout and /rosout
            console.log(count);
            ++count;
        }, 1000);
      
    });
}

//talker();

let x = () => rosnodejs.initNode('/talker_node', {onTheFly: true}).then((rosNode) => {

    /*
    let cmd_vel = rosNode.advertise('/turtle1/cmd_vel','geometry_msgs/Twist', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    });
    
    const Twist = rosnodejs.require('geometry_msgs').msg.Twist;
    const msgTwist = new Twist({
        linear: { x: 1, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 1 }
    });
    cmd_vel.publish(msgTwist);
    */

    let cmd_vel = rosNode.advertise('/New_Goal','pick_e_delivery/NewGoal', {
        queueSize: 1,
        latching: true,
        throttleMs: 9
    });
    
    const Twist = rosnodejs.require('pick_e_delivery').msg.NewGoal;
    //cmd_vel.publish({x: 9.0, y: 7.0, theta: 0.0});
    cmd_vel.publish({x: 8.0, y: 22.0, theta: 0.0});
});

let y = () => rosnodejs.initNode('/listener_node').then((rosNode) => {
    //let sub = rosNode.subscribe('/chatter', std_msgs.String,
    let sub = rosNode.subscribe('/odom', 'nav_msgs/Odometry',
        (data) => { // define callback execution
            rosnodejs.log.info('I heard: [' + JSON.stringify(data.pose.pose.position) + ']');
            sendAll(JSON.stringify(data.pose.pose.position));
        }
    );
});

let position = () => rosnodejs.initNode('/listener_node').then((rosNode) => {
    //let sub = rosNode.subscribe('/chatter', std_msgs.String,
    let sub = rosNode.subscribe('/pick_e_delivery/Pose', 'pick_e_delivery/Pose',
        (data) => { // define callback execution            
            //if( (data.transforms[0].header.frame_id == "map" && data.transforms[0].child_frame_id == "odom") ||
            //data.transforms[0].header.frame_id == "odom" && data.transforms[0].child_frame_id == "base_link")
            rosnodejs.log.info('I heard: [' + JSON.stringify(data) + ']');
            sendAll(data);
        }
    );
});


//y();
position();
//x();
app.get('/', function(req,res) {
    res.sendFile("index.html",{root: __dirname});
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