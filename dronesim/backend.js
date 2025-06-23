// server.js
const http = require('http');

let storedData = null;

const server = http.createServer((req, res) => {
  if (req.method === 'POST' && req.url === '/data') {
    let body = '';
    req.on('data', chunk => (body += chunk));
    req.on('end', () => {
      storedData = body;
      res.writeHead(200, { 'Content-Type': 'text/plain' });
      res.end('Data stored');
    });
  } else if (req.method === 'GET' && req.url === '/data') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(storedData || '{}');
  } else {
    res.writeHead(404);
    res.end('Not found');
  }
});

const PORT = 3001;
server.listen(PORT, () => {
  console.log(`Relay server running at http://localhost:${PORT}`);
});
