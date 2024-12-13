import { AppThemeProvider } from "./themes/AppThemeProvider";
import ReactDOM from "react-dom/client";
import React from "react";
import App from "./App";

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <React.StrictMode>
    <AppThemeProvider>
      <App />
    </AppThemeProvider>
  </React.StrictMode>,
);
