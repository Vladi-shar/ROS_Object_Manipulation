const express = require('express')
const app = express()
const bodyParser = require('body-parser');
const spawn = require("child_process").spawn;

app.use(express.static('public'));
app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());


app.post('/matrix', function(req, res) {
	var matrices = JSON.parse(req.body.matrices);
	const pythonProcess = spawn('python3',["A_star/Main.py", JSON.stringify(matrices[0]), JSON.stringify(matrices[1])]);
	pythonProcess.stdout.on('data', (data) => {
		//console.log(data.toString('utf8'));
    	res.send(data.toString('utf8'));
	});
	pythonProcess.stderr.on('data', (data) => {
		console.log(data.toString('utf8'));
	});

})
 
app.listen(3000)