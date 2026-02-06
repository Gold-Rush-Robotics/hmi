import {
  Button,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  Stack,
  Typography,
} from "@mui/material";
import { formatDate } from "date-fns";
import {
  formatTimeDifference,
  formatTimeDifferenceAbbreviated,
} from "../../util/util";

interface TaskHistoryDialogProps {
  open: boolean;
  onClose: () => void;
  statusHistory: {
    extendedStatus?: string;
    timestamp: Date;
  }[];
}

function TaskHistoryDialog({
  open,
  onClose,
  statusHistory,
}: TaskHistoryDialogProps) {
  const history = [...statusHistory].reverse();

  return (
    <Dialog
      open={open}
      onClose={onClose}
      maxWidth="md"
      fullWidth
      sx={{
        "& .MuiDialog-paper": {
          backgroundColor: "#000",
          borderRadius: 2,
          border: "1px solid #444",
        },
      }}
    >
      <DialogTitle>Task History ({history.length} tasks)</DialogTitle>
      <DialogContent>
        {history.length === 0 ? (
          <Typography color="text.secondary">
            No status history available
          </Typography>
        ) : (
          <Stack direction="column" spacing={2}>
            {history.map((status, index) => {
              const key = `${status.timestamp.getUTCMilliseconds()}-${status.extendedStatus}`;
              const prevStart =
                index === 0 ? undefined : history[index - 1].timestamp;
              return (
                <Stack key={key} direction="column" spacing={0.5}>
                  <Typography>{status.extendedStatus}</Typography>
                  <Typography color="text.secondary">
                    {index === 0 &&
                      `Running for ${formatTimeDifference(status.timestamp, new Date())}`}
                    {prevStart &&
                      `Ran for ${formatTimeDifferenceAbbreviated(status.timestamp, prevStart)}`}{" "}
                    (Started {formatDate(status.timestamp, "HH:mm:ss")})
                  </Typography>
                </Stack>
              );
            })}
          </Stack>
        )}
      </DialogContent>
      <DialogActions>
        <Button onClick={onClose}>Close</Button>
      </DialogActions>
    </Dialog>
  );
}

export default TaskHistoryDialog;
