import type { Dispatch, SetStateAction } from "react";
import type { GlobalStatus } from "./rosProvider";
import type { Status } from "./status";

export interface StatusSummary {
  status: Status;
}

export interface StartButton {
  status: Status;
  setStatus: Dispatch<SetStateAction<GlobalStatus[]>>;
}

export interface NavBar {
  status: Status;
  setStatus: Dispatch<SetStateAction<GlobalStatus[]>>;
}

export interface EStop {
  style: "large" | "small";
}
