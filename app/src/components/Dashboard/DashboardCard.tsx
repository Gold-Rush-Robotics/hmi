import { Box, Paper, Typography } from "@mui/material";
import { ReactNode } from "react";

interface DashboardCardProps {
  title: string;
  children: ReactNode;
}

/**
 * Reusable dashboard card component with consistent styling.
 *
 * @param props.title - The title for the card
 * @param props.children - The content to display in the card
 */
function DashboardCard({ title, children }: DashboardCardProps) {
  return (
    <Paper
      sx={{
        p: 1.5,
        bgcolor: "background.default",
        border: "1px solid",
        borderColor: "divider",
        borderRadius: "8px",
        height: "fit-content",
      }}
    >
      <Typography
        variant="h6"
        sx={{
          mb: 0.75,
          fontWeight: 600,
          color: "text.primary",
          fontSize: "0.9rem",
        }}
      >
        {title}
      </Typography>
      <Box>{children}</Box>
    </Paper>
  );
}

export default DashboardCard;
