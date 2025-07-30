import "@fontsource/inter";
import { CssBaseline, ThemeProvider } from "@mui/material";
import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import App from "./components/App";
import ROSProvider from "./components/Providers/ROSProvider";
import { theme } from "./theme";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ThemeProvider theme={theme}>
      <ROSProvider>
        <CssBaseline />
        <App />
      </ROSProvider>
    </ThemeProvider>
  </StrictMode>,
);
