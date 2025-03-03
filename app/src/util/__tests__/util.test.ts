// deepCombineObjects.test.ts
import { deepCombineObjects } from "../util"; // Adjust path as needed
import { describe, expect, it } from "vitest";

describe("deepCombineObjects", () => {
  it("should combine two simple objects", () => {
    const obj1 = { a: 1, b: 2 };
    const obj2 = { b: 3, c: 4 };
    const expected = { a: 1, b: 3, c: 4 };
    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);
  });

  it("should combine nested objects", () => {
    const obj1 = { a: { b: 1, c: 2 } };
    const obj2 = { a: { c: 3, d: 4 } };
    const expected = { a: { b: 1, c: 3, d: 4 } };
    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);
  });

  it("should combine objects with arrays", () => {
    const obj1 = { a: [1, 2], b: 3 };
    const obj2 = { a: [2, 3], c: 4 };
    const expected = { a: [1, 2, 3], b: 3, c: 4 };
    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);
  });

  it("should combine deeply nested objects and arrays", () => {
    const obj1 = {
      a: { b: { c: { list: ["x", "y"] }, d: 1 }, e: 2 },
      f: ["g", "h"],
    };
    const obj2 = {
      a: { b: { c: { list: ["y", "z"] }, e: 3 }, i: 4 },
      f: ["h", "i"],
      j: 5,
    };
    const expected = {
      a: { b: { c: { list: ["x", "y", "z"] }, d: 1, e: 3 }, e: 2, i: 4 },
      f: ["g", "h", "i"],
      j: 5,
    };
    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);
  });

  it("should handle empty objects", () => {
    const obj1 = {};
    const obj2 = { a: 1 };
    const expected = { a: 1 };
    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);

    const obj3 = { a: 1 };
    const obj4 = {};
    const expected2 = { a: 1 };
    expect(deepCombineObjects(obj3, obj4)).toEqual(expected2);

    const obj5 = {};
    const obj6 = {};
    const expected3 = {};
    expect(deepCombineObjects(obj5, obj6)).toEqual(expected3);
  });

  it("should handle overlapping array elements", () => {
    const obj1 = { a: [1, 2, 3] };
    const obj2 = { a: [3, 4, 5] };
    const expected = { a: [1, 2, 3, 4, 5] };
    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);
  });

  it("should handle complex nesting", () => {
    const obj1 = {
      level1: {
        level2: {
          level3: {
            value: ["a", "b"],
          },
        },
      },
    };
    const obj2 = {
      level1: {
        level2: {
          level3: {
            value: ["b", "c"],
            otherValue: "test",
          },
          anotherLevel3: {
            value: "another",
          },
        },
        level2_2: {
          value: "test2",
        },
      },
    };

    const expected = {
      level1: {
        level2: {
          level3: {
            value: ["a", "b", "c"],
            otherValue: "test",
          },
          anotherLevel3: {
            value: "another",
          },
        },
        level2_2: {
          value: "test2",
        },
      },
    };

    expect(deepCombineObjects(obj1, obj2)).toEqual(expected);
  });
});
