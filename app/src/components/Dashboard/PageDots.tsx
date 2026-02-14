import { Box, Stack } from "@mui/material";

interface PageDots {
  steps: number;
  index: number;
  navigateFn?: (index: number) => void;
}

/**
 * Renders a pagination of dots, with a callback function to navigate to the clicked dot.
 *
 * @param steps The number of steps to display
 * @param index The current index
 * @param navigateFn Optional. The function to navigate to the clicked dot.
 * @returns The rendered PageDots component
 */
function PageDots({ steps, index, navigateFn }: PageDots) {
  const dots = Array.from({ length: steps });
  return (
    <Stack
      direction="row"
      spacing={0.5}
      justifyContent="center"
      alignItems="center"
      sx={{ p: 1 }}
    >
      {dots.map((_, i) => (
        <Box
          key={i}
          sx={{
            height: 8,
            borderRadius: 8,
            transition: "all 300ms ease",
            cursor: "pointer",
            bgcolor: i === index ? "primary.main" : "text.disabled",
            width: i === index ? 16 : 8,
          }}
          // Navigate to the clicked dot
          onClick={() => navigateFn?.(i)}
        />
      ))}
    </Stack>
  );
}

export default PageDots;
