// Window switching keeps acquisition and the shared PoseTimeline alive. Move
// the existing timeline (not a clone) so there is only one playback state.
export function bindWorkspaceWindows({ onChange }) {
  const tabs = [...document.querySelectorAll("[data-window]")];
  const timeline = document.querySelector(".timeline-panel");
  const select = (name, focus = false) => {
    for (const tab of tabs) {
      const active = tab.dataset.window === name;
      tab.setAttribute("aria-selected", String(active));
      tab.tabIndex = active ? 0 : -1;
      document.getElementById(tab.getAttribute("aria-controls")).hidden = !active;
      if (active && focus) tab.focus();
    }
    const slot = name === "curves" ? document.querySelector("#curveTimelineSlot") : document.querySelector(".spine-panel");
    slot.append(timeline);
    onChange(name);
  };
  for (const [index, tab] of tabs.entries()) {
    tab.addEventListener("click", () => select(tab.dataset.window));
    tab.addEventListener("keydown", (event) => {
      let next;
      if (event.key === "ArrowRight") next = (index + 1) % tabs.length;
      if (event.key === "ArrowLeft") next = (index + tabs.length - 1) % tabs.length;
      if (event.key === "Home") next = 0;
      if (event.key === "End") next = tabs.length - 1;
      if (next !== undefined) { event.preventDefault(); select(tabs[next].dataset.window, true); }
    });
  }
  return select;
}
