'use strict';

const express = require('express');
const http = require('http');
const cors = require('cors');
const WebSocket = require('ws');
const request = require('request');
const session = require('express-session');
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

var app = express();
app.use(cors());
app.use(session({ //SESSION
	secret: 'LABIAGI',
	resave: true,
	/*Forza la sessione a essere risalvata nel session store, anche se mai modificata nei request*/
	saveUninitialized: false,
	/*Forza una sessione non inizializzata a essere salvata. Se false aiuta con le race conditions*/
}));

var userdb = 'http://admin:admin@localhost:5984/users/';

const port = 3000;
const FREE = 0, PICK = 1, AT_SRC = 2, DELIVERY = 3, AT_DST = 4, GOBACK = 5;

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

function publish(_x,_y,_theta,_status) {
    rosnodejs.initNode('/talker_node', {onTheFly: true}).then((rosNode) => {
        let pub_goal = rosNode.advertise('/New_Goal','pick_e_delivery/NewGoal', {
            queueSize: 1,
            latching: true,
            throttleMs: 9
        });
        pub_goal.publish({x: _x, y: _y, theta: _theta, status: _status});
    });
}

/*FUNZIONI CRUD---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

function createCRUD(db,obj) {
	let toinsert = {};
	for(let key in obj) { if(key != "id") toinsert[key] = obj[key];}
	return new Promise(function(resolve, reject){
		request({
			url: db+obj.id,
			headers: {'content-type': 'application/json'}, 
			method: 'PUT',
			body: JSON.stringify(toinsert)
		}, function(error, res, body){
			if(res.statusCode == 201) { //INSERITO
				console.log("DONE createCRUD inserito: "+201);
				resolve(obj);
			} else if(res.statusCode == 409) {//Elemento già presente
				console.log("ERROR createCRUD già presente: "+409);
				updateCRUD(db,obj).then(function(resU) {
					console.log("DONE createCRUD fatto update visto che già presente");
					reject("DONE createCRUD fatto update visto che già presente");
				}).catch(function(errU) {
					console.log("ERROR createCRUD problema in updateCRUD: "+errU);
					reject("ERROR createCRUD problema in updateCRUD: "+errU);
				});
			}
			else {
				console.log("ERROR createCRUD: "+error);
				reject("ERROR createCRUD: "+error);
			}
		});
	});
}

function readCRUD(db,obj) {
	return new Promise(function(resolve, reject){
		request({
			url: db+obj.id, 
			method: 'GET',
		}, function(error, res, body){
			if(error) {
				console.log("ERROR readCRUD: "+error);
				reject("ERROR readCRUD: "+error);
			}
			else if(res.statusCode != 200) {
				console.log("ERROR readCRUD: "+JSON.parse(body).error);
				reject(JSON.parse(body).error)
			}
			else{
				console.log("DONE readCRUD");
				resolve(JSON.parse(body));
			}
		});
	});
}

function updateCRUD(db,obj) {
	let toinsert = {};
	for(let key in obj) { if(key != "id") toinsert[key] = obj[key];}
	return new Promise(function(resolve, reject){
		readCRUD(db,obj).then(function(result) {
			toinsert["_rev"] = result._rev;	//aggiungo il _rev a obj per poter fare l'update (altrimenti da errore in accesso)
			request({//url specificato con nome dal docker compose e non localhost
				url: db+obj.id,
				method: 'PUT',
				body: JSON.stringify(toinsert), 
			}, function(error, res){
				if(error) {
					console.log("ERROR updateCRUD: "+error);
					reject("ERROR updateCRUD: "+error);
				}
				else if(res.statusCode != 201) { //non ho inserito
					console.log("ERROR updateCRUD non ho inserito: "+res.statusCode);
					reject("ERROR updateCRUD non ho inserito: "+res.statusCode);
				}
				else {
					console.log("DONE updateCRUD");
					resolve(true);
				}
			});
		}).catch(function(err){
			console.log("ERROR updateCRUD: "+err);
			reject("ERROR updateCRUD: "+err);
		});
	});
}

function deleteCRUD(db,obj) {
	return new Promise(function(resolve, reject){
		readCRUD(db,obj).then(function(result) { //prelevo i dati
			request({
				url: db+obj.id+"?rev="+result._rev,
				method: 'DELETE',
			}, function(error, res) {
				if(error) {
					console.log("ERROR deleteCRUD: "+error);
					reject("ERROR deleteCRUD: "+error);
				}
				else if(res.statusCode != 200) {
					console.log("ERROR deleteCRUD non ho cancellato: "+res.statusCode);
					reject("ERROR deleteCRUD non ho cancellato: "+res.statusCode);
				}
				else {
					console.log("DONE deleteCRUD");
					resolve(true);
				}
			});
		}).catch(function(err) {
			console.log("ERROR deleteCRUD: "+err);
			reject("ERROR deleteCRUD: "+err);
		});
	});
}

app.get('/', function(req,res) {
	if(typeof(req.session.user) != "undefined") {
		res.sendFile("index.html",{root:__dirname});
	}
	else
		res.sendFile("login.html",{root:__dirname});
});

app.get('/login', function(req,res) {
    if(typeof(req.session.user) != "undefined") {
		res.sendFile("index.html",{root:__dirname});
	}
	else
        res.sendFile("login.html",{root:__dirname});
});

app.get('/logout', function(req, res){
	req.session.destroy();
	res.redirect('/');
});

app.get('/login_action', function(req,res) {
    readCRUD(userdb, {"id": req.query.email}).then(function(res_read) {
        console.log("Utente è presente");
        if(req.query.password != res_read.password) {
            res.redirect("http://localhost:3000/login?err=Password%20errata!");
        }
        else {
            req.session.user = {"id": req.query.email, "name": res_read.name, "surname": res_read.surname, "room": res_read.room};
            //res.sendFile("index.html",{root:__dirname});
            res.redirect("/");
        }
    }).catch(function(err_read) { //non è presente, email errata
        console.log("Utente non presente");
        res.redirect("http://localhost:3000/login?err=Email%20errata%20o%20utente%20inesistente!");
    });
});

app.get('/register', function(req,res) {
    if(typeof(req.session.user) != "undefined") {
		res.sendFile("index.html",{root:__dirname});
	}
	else
        res.sendFile("register.html",{root:__dirname});
});

app.get('/register_action', function(req,res) {
    readCRUD(userdb, {"id": req.query.email}).then(function(res_read) { //verifico se l'utente già presente nel db
        //era già presente
        console.log("Utente già presente");
        res.redirect("http://localhost:3000/register?err=Utente%20già%20registrato!");
        //res.sendFile("register.html",{root:__dirname});	
    }).catch(function(err_read) { //non è presente
        console.log("Utente non presente");
        createCRUD(userdb, {"id": req.query.email, "name": req.query.name, "surname": req.query.surname, "room": req.query.room, "password": req.query.password}).then(function(res_create) {
            req.session.user = {"id": req.query.email, "name": req.query.name, "surname": req.query.surname, "room": req.query.room};
            console.log("DONE /home ho create un nuovo user: "+response.id);
            //res.sendFile("index.html",{root:__dirname});
            res.redirect("/");
        }).catch(function(err_create) {
            console.log("ERROR /home create user: "+err_create);
            res.redirect("/");
        });
    });
});

app.get('/pick', function(req,res) {
    publish(8.0,22.0,0.0,PICK);
    res.send("pick");
});

app.get('/delivery', function(req,res) {
    publish(9.0,19.0,0.0,DELIVERY);
    res.send("delivery");
});

app.get('/pick_pack', function(req,res) {
    publish(-1.0,-1.0,0.0,FREE);
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