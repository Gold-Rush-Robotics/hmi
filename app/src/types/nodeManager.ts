import { Dispatch, SetStateAction } from "react";
import { Status } from "./status";

export type NodeManager = {
  setSelectedNode: Dispatch<SetStateAction<string | null>>;
  selectedNode: string | null;
};

export type NodeItem = {
  setSelectedNode: Dispatch<SetStateAction<string | null>>;
  selection: string | null;
  name: string;
  status: Status;
};
