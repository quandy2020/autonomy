import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'node:path';

/**
 * Vite config — Dreamview uses webpack; OrbisView keeps Vite with the same
 * path aliases as Dreamview jsconfig (store / components / styles / …).
 */
export default defineConfig({
  plugins: [react()],
  publicDir: 'assets',
  resolve: {
    alias: {
      '@': path.resolve(__dirname, 'src'),
      store: path.resolve(__dirname, 'src/store'),
      components: path.resolve(__dirname, 'src/components'),
      styles: path.resolve(__dirname, 'src/styles'),
      renderer: path.resolve(__dirname, 'src/renderer'),
      utils: path.resolve(__dirname, 'src/utils'),
      assets: path.resolve(__dirname, 'assets'),
      proto_bundle: path.resolve(__dirname, 'proto_bundle'),
    },
  },
  server: {
    host: '127.0.0.1',
    port: 5173,
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
});
