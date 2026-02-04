import { differenceInSeconds } from "date-fns";

type NestedObject = {
  [key: string]: any; // Allow nested objects or arrays or literally anything
};

/**
 * Deeply combines two objects, merging nested objects and arrays.
 * When keys are identical, the second object (obj2) takes precedence and overwrites
 * values from the first object (obj1) for primitive values. Nested objects are
 * recursively merged, and arrays are combined and deduplicated.
 *
 * @param {NestedObject} obj1 The first object to combine.
 * @param {NestedObject} obj2 The second object to combine (takes precedence for identical keys).
 * @returns {NestedObject} The combined object.
 *
 * @example
 * // Simple example: obj2 overwrites obj1 for identical keys
 * const obj1 = { a: 1, b: { c: 2 } };
 * const obj2 = { a: 10, b: { d: 3 } };
 * const combined = deepCombineObjects(obj1, obj2);
 * // combined: { a: 10, b: { c: 2, d: 3 } }
 *
 * @example
 * // Complex example with nested objects and arrays
 * const obj1 = { a: { b: { c: { list: ['x', 'y'] }, d: 1 }, e: 2 }, f: ['g', 'h'] };
 * const obj2 = { a: { b: { c: { list: ['y', 'z'] }, e: 3 }, i: 4 }, f: ['h', 'i'], j: 5 };
 * const combined = deepCombineObjects(obj1, obj2);
 * // combined: { a: { b: { c: { list: [ 'x', 'y', 'z' ] }, d: 1, e: 3 }, e: 2, i: 4 }, f: [ 'g', 'h', 'i' ], j: 5 }
 */
export function deepCombineObjects(
  obj1: NestedObject,
  obj2: NestedObject,
): NestedObject {
  const result: NestedObject = { ...obj1 };

  for (const key in obj2) {
    if (obj2.hasOwnProperty(key)) {
      if (
        result.hasOwnProperty(key) &&
        typeof result[key] === "object" &&
        result[key] !== null &&
        typeof obj2[key] === "object" &&
        obj2[key] !== null
      ) {
        if (Array.isArray(result[key]) && Array.isArray(obj2[key])) {
          // Combine arrays and remove duplicates
          const combinedArray = [...result[key], ...obj2[key]];
          result[key] = [...new Set(combinedArray)];
        } else {
          // Recursive deep combine for objects
          result[key] = deepCombineObjects(
            result[key] as NestedObject,
            obj2[key] as NestedObject,
          );
        }
      } else {
        // Simple assignment for non-object/array cases
        result[key] = obj2[key];
      }
    }
  }

  return result;
}

/**
 * Formats the difference between 2 dates in `mm:ss` (or `HH:mm:ss` if it is over an hour difference).
 *
 * @param earlier The earlier date
 * @param later The later date
 * @returns A string in the form `mm:ss` or `HH:mm:ss` representing the difference in time between the two timestamps.
 */
export function formatTimeDifference(earlier: Date, later: Date) {
  const diffInSeconds = differenceInSeconds(later, earlier);

  const hours = Math.floor(diffInSeconds / 3600);
  const minutes = Math.floor((diffInSeconds % 3600) / 60);
  const seconds = diffInSeconds % 60;

  if (hours > 0) {
    return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`;
  } else {
    return `${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`;
  }
}

/**
 * Formats the difference between 2 dates in an abbreviated form.
 *
 * @param earlier The earlier date
 * @param later The later date
 * @returns A string in the form `Hh Mm Ss` (no pad, only show min when min > 0)
 */
export function formatTimeDifferenceAbbreviated(earlier: Date, later: Date) {
  const diffInSeconds = differenceInSeconds(later, earlier);

  const hours = Math.floor(diffInSeconds / 3600);
  const minutes = Math.floor((diffInSeconds % 3600) / 60);
  const seconds = diffInSeconds % 60;

  let result = "";
  if (hours > 0) {
    result += `${hours}h `;
  }
  if (minutes > 0) {
    result += `${minutes}m `;
  }
  result += `${seconds}s`;
  return result.trim();
}
