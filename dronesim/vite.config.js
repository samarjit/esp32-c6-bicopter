import { defineConfig } from 'vite'
let storedData = null;
export default defineConfig({
  build: {
    rollupOptions: {
      input: {
      app: '/testgimbal.html'
      }
    }
  },
  server: {
    open: '/index.html',
    cors: true,
  },
  // proxy: {
  //   '/data': 'http://localhost:3001',
  // },
  plugins: [
    {
      name: 'vite-relay-server',
      configureServer(server) {
        // Handle POST
        server.middlewares.use('/data', (req, res, next) => {
          if (req.method === 'POST') {
            let body = '';
            req.on('data', chunk => (body += chunk));
            req.on('end', () => {
              storedData = body;
              res.writeHead(200, { 'Content-Type': 'text/plain' });
              res.end('Stored');
            });
          } else {
            next();
          }
        });

        // Handle GET
        server.middlewares.use('/data', (req, res, next) => {
          if (req.method === 'GET') {
            res.writeHead(200, { 'Content-Type': 'application/json' });
            res.end(storedData || '{}');
          } else {
            next();
          }
        });
      },
    },
  ],
})