import { Box, Typography } from "@mui/material";

function CurrentTask() {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        p: 0.5,
      }}
    >
      <Typography variant="h6">Current Task</Typography>
      <Typography>Unknown</Typography>
    </Box>
  );
}

export default CurrentTask;
