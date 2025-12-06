import React, { useState, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import ChatKit, { askQuestionFromSelection } from '@site/src/components/ChatKit/ChatKit';
import TextSelection from '@site/src/components/TextSelection/TextSelection';
import AIAgentPanel from '@site/src/components/AIAgentPanel/AIAgentPanel';


export default function DocItemWrapper(props) {
  const [isBrowser, setIsBrowser] = useState(false);

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      setIsBrowser(true);
    }
  }, []);

  if (!isBrowser) {
    return <DocItem {...props} />;
  }

  return (
    <>
      <DocItem {...props} />
      {/* Text selection handler */}
      <TextSelection onAskQuestion={askQuestionFromSelection} />
      {/* AI Agent Panel - Explains current page */}
      <AIAgentPanel />

      {/* ChatKit only shows on docs pages */}
      <ChatKit />
    </>
  );
}


