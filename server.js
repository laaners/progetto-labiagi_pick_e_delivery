'use strict';

const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const FFplay = require("ffplay");
var player = new FFplay("./path/to/sound/file.wav"); 

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

let x = () => rosnodejs.initNode('/my_node', {onTheFly: true}).then((rosNode) => {

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
    cmd_vel.publish({x: 9.0, y: 7.0, theta: 0.0});
    //pub.publish({x: 8.0, y: 22.0, theta: 0.0});
});