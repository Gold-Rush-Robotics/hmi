import { RosMessage } from "./rosProvider";

export type Console = {
  selectedNode: string | null;
  clearSelectedNode: VoidFunction;
};

export type RosConsoleMessage = RosMessage & {
  topic: string;
};
