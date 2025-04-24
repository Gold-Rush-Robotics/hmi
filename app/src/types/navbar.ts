import { Dispatch, SetStateAction } from "react";
import { GlobalStatus } from "./rosProvider";
import { Status } from "./status";

export type StatusSummary = {
  status: Status;
};

export type StartButton = {
  status: Status;
  setStatus: Dispatch<SetStateAction<GlobalStatus[]>>;
};

export type NavBar = {
  status: Status;
  setStatus: Dispatch<SetStateAction<GlobalStatus[]>>;
};

export type EStop = {
  style: "large" | "small";
};
