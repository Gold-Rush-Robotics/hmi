import { ChevronRightRounded } from "@mui/icons-material";
import { Box, ButtonBase, Chip, Paper, Stack, Typography } from "@mui/material";
import { ReactNode } from "react";

interface DashboardCardProps {
  title: string;
  children: ReactNode;
  action?: DashboardCardAction;
}

interface DashboardCardAction {
  onClick: () => void;
  label: string;
}

/**
 * Reusable dashboard card component with consistent styling.
 *
 * @param props.title - The title for the card
 * @param props.children - The content to display in the card
 */
function DashboardCard({ title, children, action }: DashboardCardProps) {
  const paperContent = (
    <Paper
      sx={{
        p: 1.5,
        bgcolor: "background.default",
        border: "1px solid",
        borderColor: "divider",
        borderRadius: "8px",
        height: "fit-content",
        width: "100%",
      }}
    >
      <Stack
        direction="row"
        justifyContent="space-between"
        alignItems="center"
        mb={0.75}
      >
        <Typography
          variant="h6"
          sx={{
            fontWeight: 600,
            color: "text.primary",
            fontSize: "0.9rem",
          }}
        >
          {title}
        </Typography>
        {action && (
          <Chip
            label={action.label}
            size="small"
            deleteIcon={<ChevronRightRounded />} // hack for right-aligned icon
            onDelete={() => {}} // hack for right-aligned icon
            onClick={action.onClick}
            sx={{ fontSize: "0.75rem", height: 24, pl: 0.5, pr: 0.3 }}
          />
        )}
      </Stack>
      <Box>{children}</Box>
    </Paper>
  );

  // If action is provided, wrap the paper content in a button base (whole card acts as a button)
  if (action) {
    return (
      <ButtonBase
        onClick={action.onClick}
        sx={{
          width: "100%",
          borderRadius: "8px",
          overflow: "hidden",
          textAlign: "left", // Ensure content is left-aligned
        }}
      >
        {paperContent}
      </ButtonBase>
    );
  }

  return paperContent;
}

export default DashboardCard;
