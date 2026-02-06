import type { Dispatch, SetStateAction } from "react";
import type { Status } from "./status";

export type WSHistory = Record<string, NodeHistory>;

export type NodeHistory = Record<string, RosMessage[]>;

export interface RosMessage {
  timestamp: Date;
  message: object | string;
}

export type DiscoveredNodes = Record<string, DiscoveredNodeInfo>;

export interface DiscoveredNodeInfo {
  publishers: DiscoveredTopic[];
  service_clients: DiscoveredTopic[];
  service_servers: DiscoveredTopic[];
  subscribers: DiscoveredTopic[];
  status: Status;
}

export interface DiscoveredTopic {
  topic: string;
  type: RosType;
}

export type RosType = string;

export type RosResponse = RosPublishResponse | RosServiceRespone;

export interface RosPublishResponse {
  op: "publish";
  topic: string;
  msg: unknown;
}

export interface RosServiceRespone {
  op: "service_response";
  service: RosType;
  values: unknown;
  result: boolean;
}

export type RosNodeInfo = Record<string, RosNodeConnections>;

export interface RosNodeConnections {
  publishers: RosNodeConnection[];
  service_clients: RosNodeConnection[];
  service_servers: RosNodeConnection[];
  subscribers: RosNodeConnection[];
}

export interface RosNodeConnection {
  topic: string;
  type: string;
}

export interface GlobalStatusContextType {
  globalStatusHistory: GlobalStatus[];
  setGlobalStatusHistory: Dispatch<SetStateAction<GlobalStatus[]>>;
}

export interface GlobalStatus {
  timestamp: Date;
  status: Status;
  extendedStatus?: string;
}

export interface RosCommunicationContext {
  sendRaw: (message: object) => void;
  advertise: (topic: string, type: string) => void;
  callService: (service: string, args?: object[]) => void;
  publish: (topic: string, msg: object | string) => void;
  subscribe: (topic: string, type: RosType) => void;
}

export type RosDashboardScreenItems = Record<
  string,
  Record<string, RosDashboardItemData>
>;

export type RosDashboardItemData =
  | {
      screen: string;
      id: string;
      type: "card";
      data: RosDashboardCard;
    }
  | {
      screen: string;
      id: string;
      type: "bar";
      data: RosDashboardBar;
    }
  | {
      screen: string;
      id: string;
      type: "color";
      data: RosDashboardColor;
    };

export interface RosDashboardCard {
  title: string;
  content?: string;
}

export interface RosDashboardBar {
  title: string;
  content?: string;
  value: number;
  min?: number;
  max?: number;
}

export interface RosDashboardColor {
  title: string;
  content?: string;
  color: string;
}
