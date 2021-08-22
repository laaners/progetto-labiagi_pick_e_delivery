'use strict';

const express = require('express');
const http = require('http');
const cors = require('cors');
const WebSocket = require('ws');
const request = require('request');
const session = require('express-session');
const cookie = require('cookie');
const cookieParser = require('cookie-parser')
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

var app = express();
app.use(cors());
const sessionOpts = {
	secret: 'LABIAGI',
	resave: false,
	saveUninitialized: true,
};
const sess = session(sessionOpts);
app.use(sess);
/*
app.use(session({ //SESSION
	secret: 'LABIAGI',
	resave: true,
	//Forza la sessione a essere risalvata nel session store, anche se mai modificata nei request
	saveUninitialized: false,
	//Forza una sessione non inizializzata a essere salvata. Se false aiuta con le race conditions
}));
*/


const port = 3000;
const FREE = 0, PICK = 1, AT_SRC = 2, DELIVERY = 3, AT_DST = 4, GOBACK = 5;

var usersdb = 'http://admin:admin@localhost:5984/users/';
var roomsdb = 'http://admin:admin@localhost:5984/rooms/';

/*
USER: {
	"id": "",
	"name": "",
	"surname": "",
	"room": "",
	"password": ""
}
curl -X GET http://admin:admin@127.0.0.1:5984/users/_all_docs?include_docs=true

ROOM: {
	"id": "",
	"x": "",
	"y": "",
	"users": []
}
curl -X GET http://admin:admin@127.0.0.1:5984/rooms/_all_docs?include_docs=true
*/

var status = FREE;
var sender = "none";
var receiver = "none";
var tosend = {"position": null, "goal": null, "online": []};

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

function publish(_x,_y,_theta,_command,_user) {
    rosnodejs.initNode('/talker_node', {onTheFly: true}).then((rosNode) => {
        let pub_goal = rosNode.advertise('/New_Goal','pick_e_delivery/NewGoal', {
            queueSize: 1,
            latching: true,
            throttleMs: 9
        });
        pub_goal.publish({x: _x, y: _y, theta: _theta, command: _command, user: _user});
    });
}

function int2Status(n) {
	switch(n) {
		case 0: return "FREE"; break;
		case 1: return "PICK"; break;
		case 2: return "AT_SRC"; break;
		case 3: return "DELIVERY"; break;
		case 4: return "AT_DST"; break;
		case 5: return "GOBACK"; break;
		default: return "STATO NON RICONOSCIUTO"; break;
	}
}

function string2Command(s) {
	switch(s) {
		case "FREE": return FREE; break;
		case "PICK": return PICK; break;
		case "DELIVERY": return DELIVERY; break;
		case "GOBACK": return GOBACK; break;
		default: return "COMANDO NON RICONOSCIUTO"; break;
	}
}

rosnodejs.initNode('/talker_node', {onTheFly: true}).then((rosNode) => {
    //let tosend = {"position": null, "goal": null, "online": []};

    let sub_pos = rosNode.subscribe('/pick_e_delivery/Pose', 'pick_e_delivery/Pose',
        (data) => { // define callback execution            
            tosend.position = data;
			status = data.status;
			sender = data.sender;
			receiver = data.receiver;
            //rosnodejs.log.info(JSON.stringify(tosend.position));
			//rosnodejs.log.info(int2Status(status));
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
    //sendAll(tosend);
});

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
    readCRUD(usersdb, {"id": req.query.email}).then(function(res_readu) {
        console.log("DONE /login_action l'utente è presente");
        if(req.query.password != res_readu.password) {
            res.redirect("http://localhost:3000/login?err=Password%20errata!");
        }
        else {
            req.session.user = {"id": req.query.email, "name": res_readu.name, "surname": res_readu.surname, "room": res_readu.room, "password": res_readu.password};
            //res.sendFile("index.html",{root:__dirname});
			console.log(req.session.user);
            res.redirect("/");
        }
    }).catch(function(err_readu) { //non è presente, email errata
        console.log("ERROR /login_action utente non presente: "+err_readu);
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
    readCRUD(usersdb, {"id": req.query.email}).then(function(res_readu) { //verifico se l'utente già presente nel db
        //era già presente
        console.log("DONE /action_register utente già presente");
        res.redirect("http://localhost:3000/register?err=Utente%20già%20registrato!");
        //res.sendFile("register.html",{root:__dirname});	
    }).catch(function(err_readu) { //non è presente
        console.log("ERROR /register_action utente non presente: "+err_readu);
        createCRUD(usersdb, {"id": req.query.email, "name": req.query.name, "surname": req.query.surname, "room": req.query.room, "password": req.query.password}).then(function(res_createu) {
            req.session.user = {"id": req.query.email, "name": req.query.name, "surname": req.query.surname, "room": req.query.room, "password": req.query.password};
            console.log("DONE /register_action ho creato un nuovo user: "+req.query.id);
			readCRUD(roomsdb, {"id": req.query.room}).then(function(res_readr) { //La room esiste in DB
				let new_users = res_readr.users;
				new_users.push(req.query.email);
				let obj = {
					"id": res_readr._id,
					"x": res_readr.x,
					"y": res_readr.y,
					"users": new_users
				};
				updateCRUD(roomsdb, obj).then(function(res_updater) { //...QUINDI AGGIORNO QUELL'ITINERARIO IN DB (userN+1)...
					console.log("DONE /register_action update rooms: "+obj.id);
					res.redirect("/");
				}).catch(function(err_updater) { console.log("ERROR /register_action update room: "+err_updater); res.redirect("/"); });
			}).catch(function(err_readr) { //Se la room non esiste in DB, errore
				console.log("ERROR /register_action read rooms: "+err_readr);
				res.redirect("/");
			});
        }).catch(function(err_createu) {
            console.log("ERROR /register_action create user: "+err_createu);
            res.redirect("/");
        });
    });
});

app.get('/user_data', function(req, res) {
	readCRUD(usersdb, {"id": req.session.user.id}).then(function(res_readu) {
		req.session.user = {"id": res_readu._id, "name": res_readu.name, "surname": res_readu.surname, "room": res_readu.room, "password": res_readu.password};
		console.log("DONE /user_data ho fatto la read e sto mandando: "); 
		console.log(req.session.user);
		res.send(req.session.user);
	}).catch(function(err_readu) {
		console.log("ERROR /user_data: "+err_readu);
		res.send("ERROR /user_data: "+err_readu);
	});
});

app.get('/move_robot', function(req,res) {
	let ACTION = string2Command(req.query.action);
	if(req.query.user != req.session.user.id && req.query.password != req.session.user.password) {
		console.log("ERROR /move_robot attacco melevolo")
		res.send("ERROR /move_robot attacco melevolo");
		return;
	}
	if(ACTION == FREE) {
		if((status == AT_SRC) && (req.query.user == sender)) console.log("FREE src valido");
		else if((status == PICK) && (req.query.user == sender)) console.log("FREE pick src valido");
		else if(status == AT_DST && req.query.user == receiver) console.log("FREE dst valido");
		else {
			console.log("FREE non valido: "+req.query.user);
			res.send("ERROR /move_robot free non valido");
			return;
		}
	}
	readCRUD(usersdb,{"id": req.query.goal}).then(function(res_readu) {
		console.log("DONE /move_robot readu");
		let room = res_readu.room;
		readCRUD(roomsdb,{"id": room}).then(function(res_readr) {
			console.log("DONE /move_robot readr");
			publish(res_readr.x,res_readr.y,0.0,ACTION,req.query.goal);
			res.send("pick");		
		}).catch(function(err_readr) {
			console.log("ERROR /move_robot readr: "+err_readr);
			res.send("useless data");
		})
	}).catch(function(err_readu) {
		console.log("ERROR /move_robot readu: "+err_readu);
		res.send("useless data");
	});
});

app.get('/map', function(req,res) {
    res.sendFile("april_tag_scale.png",{root: __dirname});
});

app.get('/addio', function(req,res) {
	console.log(ONLINE);
	res.send("ciao");
});

const httpServer = http.createServer(app);
const ws = new WebSocket.Server({server: httpServer});

var CLIENTS = [];
//var ONLINE = [];

ws.on('connection', function(conn,req) {
    CLIENTS.push(conn);
	sess(req, {}, () => {
		console.log('Session is parsed!');
		//ONLINE.push(req.session.user.id);
		//console.log(ONLINE);
		tosend.online.push(req.session.user.id);
		console.log(tosend.online);
	});

    conn.on('message', function(message) {
        console.log("Ricevuto: "+message);
        sendAll(message);
    });

    conn.on('close', function() {
        console.log("Connessione chiusa");
        CLIENTS.splice(CLIENTS.indexOf(conn),1);
		sess(req, {}, () => {
			console.log('Session is parsed!');
			//ONLINE.splice(ONLINE.indexOf(req.session.user.id),1);
			//console.log(ONLINE);
			tosend.online.splice(tosend.online.indexOf(req.session.user.id),1);
			console.log(tosend.online);
		});
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