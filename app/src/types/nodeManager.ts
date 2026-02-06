import type { Dispatch, SetStateAction } from "react";
import type { Status } from "./status";

export interface NodeManager {
  setSelectedNode: Dispatch<SetStateAction<string | null>>;
  selectedNode: string | null;
}

export interface NodeItem {
  setSelectedNode: Dispatch<SetStateAction<string | null>>;
  selection: string | null;
  name: string;
  status: Status;
}
