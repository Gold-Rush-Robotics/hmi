import { useContext } from "react";
import {
  WSHistoryContext,
  DiscoveredNodesContext,
  ROSCommunicationContext,
} from "../ROSProvider";

/**
 * A react component for exposing the state from `ROSProvider.tsx`. To use
 * this component, place it anywhere inside of a ROSProvider. This component
 * will expose:
 *
 * - `WSHistoryContext`: via data-testid="ws-history" as a JSON string (must
 * JSON.parse())
 * - `DiscoveredNodesContext`: via data-testid="discovered-nodes" as a JSON
 * string (must JSON.parse())
 * - Some hardcoded methods for being able to test `sendRaw`, `advertise`,
 * `publish`, and `subscribe` (see this component for more info).
 *
 * @returns divs with testids containing the exposed data
 */
export function ExposeROSProvider() {
  const WSHistory = useContext(WSHistoryContext);
  const DiscoveredNodes = useContext(DiscoveredNodesContext);

  const WSHistoryString = JSON.stringify(WSHistory);
  const DiscoveredNodesString = JSON.stringify(DiscoveredNodes);
  const { sendRaw, advertise, publish, subscribe } = useContext(
    ROSCommunicationContext,
  );

  return (
    <div data-testid="exposed-ros-context">
      <div data-testid="ws-history">{WSHistoryString}</div>
      <div data-testid="discovered-nodes">{DiscoveredNodesString}</div>
      {/* Below here are some methods to be able to send data (note the data is hardcoded) */}
      <button
        data-testid="send-raw"
        onClick={() => sendRaw({ test: "data" })}
      ></button>
      <button
        data-testid="advertise"
        onClick={() => advertise("/advertise_topic", "std_msgs/msg/String")}
      ></button>
      <button
        data-testid="publish"
        onClick={() => {
          publish("/publish_topic", { data: "test message" });
          publish("/publish_topic", "string_should_work_too");
        }}
      ></button>
      <button
        data-testid="subscribe"
        onClick={() => subscribe("/subscribe_topic", "std_msgs/msg/String")}
      ></button>
    </div>
  );
}
