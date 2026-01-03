import React from 'react';
import DefaultLayout from '@theme-original/Layout';
import FloatingChatbot from '@site/src/components/FloatingChatbot';

export default function Layout(props) {
  return (
    <>
      <DefaultLayout {...props}>
        {props.children}
        <FloatingChatbot />
      </DefaultLayout>
    </>
  );
}