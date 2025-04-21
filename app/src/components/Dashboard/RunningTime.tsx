import { Box, Typography } from "@mui/material";

function RunningTime() {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        p: 0.5,
      }}
    >
      <Typography variant="h6">Running Time</Typography>
      <Typography>Started 17:10:04</Typography>
      <Typography>Running for 02:14</Typography>
    </Box>
  );
}

export default RunningTime;
