import { defineConfig } from 'vite';
import { resolve } from 'path';

export default defineConfig({
  // Multi-page app configuration — each robot page is a separate entry
  build: {
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        rrrLamp: resolve(__dirname, 'src/pages/rrr-lamp.html'),
      },
    },
  },
  // Serve the public/ directory as static assets
  publicDir: 'public',
});
