import type { Dispatch, SetStateAction } from "react";
import type { RosMessage } from "./rosProvider";

export interface Console {
  selectedNode: string | null;
  clearSelectedNode: VoidFunction;
}

export interface RosConsoleMessage extends RosMessage {
  topic: string;
}

export interface ConsoleFilters {
  topicMap: Map<string, boolean>;
  disabledTopics: string[];
  setDisabledTopics: Dispatch<SetStateAction<string[]>>;
}
