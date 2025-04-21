import { Dispatch, SetStateAction } from "react";
import { Status } from "./status";

export type StatusSummary = {
  status: Status;
};

export type StartButton = {
  status: Status;
  setStatus: Dispatch<SetStateAction<Status>>;
};

export type NavBar = {
  status: Status;
  setStatus: Dispatch<SetStateAction<Status>>;
};

export type EStop = {
  style: "large" | "small";
  setStatus: Dispatch<SetStateAction<Status>>;
};
