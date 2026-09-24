import { defineConfig } from 'vite'

const target = process.env.ROBOT_URL || 'http://192.168.178.55'
const robot = new URL(target)
if (!['http:', 'https:'].includes(robot.protocol) || robot.username || robot.password || robot.pathname !== '/' || robot.search || robot.hash) {
  throw new Error('ROBOT_URL must be an http(s) origin, for example http://192.168.1.50')
}

export default defineConfig({
  server: {
    host: '127.0.0.1',
    port: 5173,
    strictPort: true,
    proxy: {
      '/api': {
        target: robot.origin,
        changeOrigin: true,
        rewrite: path => path.replace(/^\/api(?=\/|$)/, ''),
      },
    },
  },
})
