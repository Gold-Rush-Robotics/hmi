import { Dispatch, SetStateAction } from "react";
import { Status } from "./status";

export type WSHistory = {
  [key: string]: NodeHistory;
};

export type NodeHistory = {
  [key: string]: RosMessage[];
};

export type RosMessage = {
  timestamp: Date;
  message: object;
};

export type DiscoveredNodes = {
  [key: string]: DiscoveredNodeInfo;
};

export type DiscoveredNodeInfo = {
  action_clients: DiscoveredTopic[];
  action_servers: DiscoveredTopic[];
  publishers: DiscoveredTopic[];
  service_clients: DiscoveredTopic[];
  service_servers: DiscoveredTopic[];
  subscribers: DiscoveredTopic[];
};

export type DiscoveredTopic = {
  topic: string;
  type: RosType;
};

export type RosType = "std_msgs/msg/String" | string;

export type RosResponse = {
  op: string;
  topic: string;
  msg: any;
};

export type RosNodeInfo = {
  [key: string]: RosNodeConnections;
};

export type RosNodeConnections = {
  action_clients: RosNodeConnection[];
  action_servers: RosNodeConnection[];
  publishers: RosNodeConnection[];
  service_clients: RosNodeConnection[];
  service_servers: RosNodeConnection[];
  subscribers: RosNodeConnection[];
};

export type RosNodeConnection = {
  topic: string;
  type: string;
};

export type GlobalStatusContextType = {
  globalStatus: Status;
  setGlobalStatus: Dispatch<SetStateAction<Status>>;
};

export type RosCommunicationContext = {
  sendRaw: (message: object) => void;
  advertise: (topic: string, type: string) => void;
  publish: (topic: string, msg: object | string) => void;
  subscribe: (topic: string, type: RosType) => void;
};
