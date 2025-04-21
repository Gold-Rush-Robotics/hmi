import { Box, Typography } from "@mui/material";

function NodeHealth() {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        p: 0.5,
      }}
    >
      <Typography variant="h6">Node Health</Typography>
      <Typography>5 discovered nodes</Typography>
      <Typography sx={{ pl: 1 }}> - 4 running</Typography>
      <Typography sx={{ pl: 1 }}> - 1 stopped</Typography>
      <Typography>14 discovered topics</Typography>
      <Typography sx={{ pl: 1 }}>- 18 msgs/sec</Typography>
    </Box>
  );
}

export default NodeHealth;
