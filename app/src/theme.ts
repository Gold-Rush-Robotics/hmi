import { createTheme } from "@mui/material/styles";

export const theme = createTheme({
  palette: {
    mode: "dark",
    primary: {
      main: "#b89659",
      light: "#d4b17a",
      dark: "#9a7a3a",
      contrastText: "#000",
    },
    secondary: {
      main: "#0d553f",
      light: "#1a7a5f",
      dark: "#0a3f2f",
      contrastText: "#fff",
    },
    background: {
      default: "#121212",
      paper: "#1e1e1e",
    },
    text: {
      primary: "#ffffff",
      secondary: "#b3b3b3",
    },
    divider: "#333333",
    action: {
      active: "#ffffff",
      hover: "rgba(255, 255, 255, 0.08)",
      selected: "rgba(255, 255, 255, 0.16)",
      disabled: "rgba(255, 255, 255, 0.3)",
      disabledBackground: "rgba(255, 255, 255, 0.12)",
    },
  },
  typography: {
    fontFamily: "Inter, system-ui, Avenir, Helvetica, Arial, sans-serif",
    h4: {
      fontWeight: 600,
    },
    subtitle1: {
      fontSize: "0.875rem",
      fontWeight: 600,
    },
    body2: {
      fontSize: "0.75rem",
    },
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: {
        body: {
          backgroundColor: "#121212",
          color: "#ffffff",
        },
        ":root": {
          "--button-bg": "#2a2a2a", // Custom CSS variable for button background
        },
      },
    },
    MuiPaper: {
      styleOverrides: {
        root: {
          backgroundColor: "#1e1e1e",
        },
      },
    },
    MuiCard: {
      styleOverrides: {
        root: {
          backgroundColor: "#1e1e1e",
        },
      },
    },
    // Optimize components for small screens
    MuiChip: {
      styleOverrides: {
        root: {
          height: "24px",
          fontSize: "0.7rem",
        },
        label: {
          paddingLeft: "8px",
          paddingRight: "8px",
        },
      },
    },
    MuiToggleButton: {
      styleOverrides: {
        root: {
          padding: "4px 8px",
          fontSize: "0.75rem",
        },
        sizeSmall: {
          padding: "2px 4px",
          fontSize: "0.7rem",
        },
      },
    },
  },
  // Add responsive breakpoints for small screens
  breakpoints: {
    values: {
      xs: 0,
      sm: 600,
      md: 960,
      lg: 1280,
      xl: 1920,
    },
  },
});
