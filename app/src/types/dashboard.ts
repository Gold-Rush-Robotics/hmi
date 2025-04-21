import { Dispatch, SetStateAction } from "react";
import { Status } from "./status";

export type Dashboard = {
  setStatus: Dispatch<SetStateAction<Status>>;
};
