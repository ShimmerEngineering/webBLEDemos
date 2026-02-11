chrome.runtime.onMessage.addListener((message, sender, sendResponse) => {
    if (message.type === "VIDEO_TIME") {
        chrome.runtime.sendMessage(message);
    }
});

chrome.action.onClicked.addListener(() => {
    chrome.tabs.create({ url: "shimmercompanion.html" });
});