import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import "./css/index.css";
import App from "./components/App";
import "@fontsource/inter";
import { CssBaseline } from "@mui/joy";
import ROSProvider from "./components/Providers/ROSProvider";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ROSProvider>
      <CssBaseline />
      <App />
    </ROSProvider>
  </StrictMode>,
);
