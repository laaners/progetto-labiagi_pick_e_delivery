var test = ["kirbyalessio@yahoo.it", "aa@gmail.com"];
let less_users = test;
console.log(test);
if(less_users.includes("aa@gmail.com") && less_users.length <= 1) less_users = [];
else {
    less_users.splice(less_users.indexOf("aa@gmail.com"),1);
}
console.log(less_users);	