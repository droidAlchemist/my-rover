import { defineConfig } from "vite";
import react from "@vitejs/plugin-react-swc";

export default defineConfig({
  plugins: [react()],
  css: {
    preprocessorOptions: {
      scss: {
        api: "modern-compiler", // or "modern"
      },
    },
  },
  resolve: {
    alias: {
      "@/app": "/src/app",
      "@/assets": "/src/assets",
      "@/components": "/src/components",
      "@/features": "/src/features",
      "@/hooks": "/src/hooks",
      "@/pages": "/src/pages",
      "@/routes": "/src/routes",
      "@/themes": "/src/themes",
      "@/styles": "/src/styles",
      "@/utils": "/src/utils",
      "@/services": "/src/services",
      "@/config": "/src/config",
      "@/types": "/src/types",
      "@/store": "/src/store",
      "@/layout": "/src/layout",
      "@/data": "/src/data",
      "@/helper": "/src/helper",
      "@/i18n": "/src/i18n",
    },
  },
  define: {
    // Some libraries use the global object, even though it doesn't exist in the browser.
    // Alternatively, we could add `<script>window.global = window;</script>` to index.html.
    // https://github.com/vitejs/vite/discussions/5912
    global: {},
    "process.env": {},
  },
});
