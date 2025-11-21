import { Box, Stack } from "@mui/material";

type PageDots = {
  steps: number;
  index: number;
};

function PageDots({ steps, index }: PageDots) {
  const dots = Array.from({ length: steps });
  return (
    <Stack
      direction="row"
      spacing={0.5} // Space between the dots
      justifyContent="center"
      alignItems="center"
      sx={{ p: 1 }}
    >
      {dots.map((_, i) => (
        <Box
          key={i}
          sx={{
            // Base size for all dots
            height: 8,
            borderRadius: 8,
            transition: "all 300ms ease",
            cursor: "pointer",

            // Conditional styling for the active dot
            bgcolor: i === index ? "primary.main" : "text.disabled",

            // Optionally make the active dot larger
            width: i === index ? 16 : 8,
          }}
          // You could add an onClick handler here if the dots should be navigable
          // onClick={() => console.log(`Clicked dot ${index}`)}
        />
      ))}
    </Stack>
  );
}

export default PageDots;
