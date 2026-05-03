import { defineConfig } from 'vite';

export default defineConfig({
  base: '/JINX-Joint_Inverse_N-dimensional_eXplorer/',
  // Multi-page app configuration — each robot page is a separate entry
  build: {
    rollupOptions: {
      input: {
        main: 'index.html',
        rrr: 'src/pages/rrr-lamp.html',
        scara: 'src/pages/scara.html',
        welder: 'src/pages/welder.html',
        math: 'src/pages/math-dashboard.html',
      },
    },
  },
  // Serve the public/ directory as static assets
  publicDir: 'public',
});
