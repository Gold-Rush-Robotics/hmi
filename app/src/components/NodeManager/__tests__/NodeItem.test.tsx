import { cleanup, render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { afterEach, describe, expect, it, vi } from "vitest";
import { Status } from "../../../types/status";
import NodeItem from "../NodeItem";

describe("NodeItem", () => {
  // Without this, every render() from previous tests stays in the environment
  afterEach(() => {
    cleanup();
  });

  it("renders a button with the name of the node", async () => {
    const clickFn = vi.fn();
    render(
      <NodeItem
        setSelectedNode={clickFn}
        selection={null}
        name={"My Node"}
        status={Status.OK}
      />,
    );
    expect(await screen.findByText("My Node")).toBeInTheDocument();
    expect(clickFn).toHaveBeenCalledTimes(0);
  });

  it("calls setSelected when clicked", async () => {
    const clickFn = vi.fn();
    render(
      <NodeItem
        setSelectedNode={clickFn}
        selection={null}
        name={"My Node"}
        status={Status.OK}
      />,
    );
    await userEvent.click(screen.getByText("My Node"));
    expect(clickFn).toHaveBeenCalled();
  });

  it("is selected when the selection matches its name", () => {
    const clickFn = vi.fn();
    render(
      <NodeItem
        setSelectedNode={clickFn}
        selection={"My Node"}
        name={"My Node"}
        status={Status.OK}
      />,
    );
    const component = screen.getByText("My Node");
    expect(component.style.backgroundColor === "#eee");
  });

  it("is not selected when the selection is something else", () => {
    const clickFn = vi.fn();
    render(
      <NodeItem
        setSelectedNode={clickFn}
        selection={"Another Node"}
        name={"My Node"}
        status={Status.OK}
      />,
    );
    const component = screen.getByText("My Node");
    expect(component.style.backgroundColor !== "#eee");
  });

  it("is not selected when the selection is null", () => {
    const clickFn = vi.fn();
    render(
      <NodeItem
        setSelectedNode={clickFn}
        selection={null}
        name={"My Node"}
        status={Status.OK}
      />,
    );
    const component = screen.getByText("My Node");
    expect(component.style.backgroundColor !== "#eee");
  });
});
