import React from "react";
import ChatbotWidget from "../components/ChatbotWidget/ChatbotWidget";
// Root component to wrap the entire Docusaurus app
const Root = ({ children }) => {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
};

export default Root;
