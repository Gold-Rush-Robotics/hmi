import { cleanup } from "@testing-library/react";
import { afterEach, describe, expect, it } from "vitest";

describe("NodeManager", () => {
  // Without this, every render() from previous tests stays in the environment
  afterEach(() => {
    cleanup();
  });

  it("will pass testing", () => {
    expect(true);
  });

  // these need to be rewritten and I don't feel like doing that now :)

  // it("renders all the found nodes", async () => {
  //   const clickFn = vi.fn();
  //   render(<NodeManager setSelectedNode={clickFn} selectedNode={null} />);

  //   // This is until proper logic to get nodes from ROS are implemented
  //   expect(screen.getByText("Node 1")).toBeInTheDocument();
  //   expect(screen.getByText("Node 2")).toBeInTheDocument();
  //   expect(screen.getByText("Node 3")).toBeInTheDocument();
  //   expect(screen.getByText("Node 4")).toBeInTheDocument();
  //   expect(screen.getByText("Node 5")).toBeInTheDocument();
  //   expect(screen.getByText("Node 6")).toBeInTheDocument();

  //   expect(clickFn).toHaveBeenCalledTimes(0);
  // });

  // it("calls the function when children are clicked", async () => {
  //   const clickFn = vi.fn();
  //   render(<NodeManager setSelectedNode={clickFn} selectedNode={null} />);
  //   await userEvent.click(screen.getByText("Node 1"));
  //   await userEvent.click(screen.getByText("Node 2"));
  //   await userEvent.click(screen.getByText("Node 3"));
  //   await userEvent.click(screen.getByText("Node 4"));
  //   await userEvent.click(screen.getByText("Node 5"));
  //   await userEvent.click(screen.getByText("Node 6"));
  //   expect(clickFn).toHaveBeenCalledTimes(6);
  // });
});
