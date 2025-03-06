import { Dispatch, SetStateAction } from "react";
import { RosMessage } from "./rosProvider";

export type Console = {
  selectedNode: string | null;
  clearSelectedNode: VoidFunction;
};

export type RosConsoleMessage = RosMessage & {
  topic: string;
};

export type ConsoleFilters = {
  topicMap: Map<string, boolean>;
  disabledTopics: string[];
  setDisabledTopics: Dispatch<SetStateAction<string[]>>;
};
