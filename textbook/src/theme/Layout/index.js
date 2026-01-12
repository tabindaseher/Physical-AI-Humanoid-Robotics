import React from 'react';
import Layout from '@theme-original/Layout';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
        <ChatbotWidget />
      </Layout>
    </>
  );
}