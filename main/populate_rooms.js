const request = require('request');

const rooms = {"total_rows":8,"offset":0,"rows":[
    {"id":"A","key":"A","value":{"rev":"2-0625968f6039eb21969ea2e358536c76"},"doc":{"_id":"A","_rev":"2-0625968f6039eb21969ea2e358536c76","x":8.35530090332,"y":7.64831256866,"users":[]}},
    {"id":"B","key":"B","value":{"rev":"1-f31a0257e2c6132aa42ad28cbfc41f8f"},"doc":{"_id":"B","_rev":"1-f31a0257e2c6132aa42ad28cbfc41f8f","x":7.8879327774,"y":11.8971090317,"users":[]}},
    {"id":"C","key":"C","value":{"rev":"1-adac585344c6a85e03e123d68422f69e"},"doc":{"_id":"C","_rev":"1-adac585344c6a85e03e123d68422f69e","x":7.42056560516,"y":16.4008331299,"users":[]}},
    {"id":"D","key":"D","value":{"rev":"1-a35587ce191a4e75422196fc62f488f8"},"doc":{"_id":"D","_rev":"1-a35587ce191a4e75422196fc62f488f8","x":7.1231508255,"y":21.074508667,"users":[]}},
    {"id":"E","key":"E","value":{"rev":"1-9483eb4acb1a036cf32e9fbcaec77192"},"doc":{"_id":"E","_rev":"1-9483eb4acb1a036cf32e9fbcaec77192","x":11.5418977737,"y":21.1594848633,"users":[]}},
    {"id":"F","key":"F","value":{"rev":"1-88d3fb4890c0895287c9d04baad4c999"},"doc":{"_id":"F","_rev":"1-88d3fb4890c0895287c9d04baad4c999","x":11.0745296478,"y":16.6982498169,"users":[]}},
    {"id":"G","key":"G","value":{"rev":"1-adefc689d2f0fc05698b809489754e7a"},"doc":{"_id":"G","_rev":"1-adefc689d2f0fc05698b809489754e7a","x":12.1000356674,"y":11.9346723557,"users":[]}},
    {"id":"H","key":"H","value":{"rev":"1-60c1544dff348708af61a14c8ec0353a"},"doc":{"_id":"H","_rev":"1-60c1544dff348708af61a14c8ec0353a","x":12.2105884552,"y":7.4757027626,"users":[]}},
    {"id":"Z","key":"H","value":{"rev":"1-60c1544dff348708af61a14c8ec0353a"},"doc":{"_id":"H","_rev":"1-60c1544dff348708af61a14c8ec0353a","x":12.2105884552,"y":7.4757027626,"users":[]}}
    ]}    

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
			}
			else {
				console.log("ERROR createCRUD: "+error);
				reject("ERROR createCRUD: "+error);
			}
		});
	});
}

var rooms_map = rooms.rows.map(x=> obj = {
    "id": x.id,
    "x": x.doc.x,
    "y": x.doc.y,
    "users": x.doc.users
});
for(let i = 0; i < rooms_map.length; i++) {
    console.log(rooms_map[i]);
    createCRUD('http://admin:admin@localhost:5984/rooms/',rooms_map[i]).then(() => console.log("Stanza "+rooms_map[i].id+" creata"))
        .catch((err)=>console.log("Errore stanza "+rooms_map[i].id+": "+err));
}