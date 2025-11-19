import React, { Component, ErrorInfo, ReactNode } from 'react';

interface Props {
  children: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
}

export class ErrorBoundary extends Component<Props, State> {
  public state: State = {
    hasError: false,
    error: null,
    errorInfo: null
  };

  public static getDerivedStateFromError(error: Error): State {
    return { hasError: true, error, errorInfo: null };
  }

  public componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.error("Uncaught error:", error, errorInfo);
    this.setState({ error, errorInfo });
  }

  public render() {
    if (this.state.hasError) {
      return (
        <div className="p-8 bg-red-50 min-h-screen text-red-900 font-sans">
          <h1 className="text-2xl font-bold mb-4">Something went wrong</h1>
          <div className="bg-white p-4 rounded shadow border border-red-200 overflow-auto">
            <h2 className="font-semibold text-red-700 mb-2">{this.state.error?.toString()}</h2>
            <details className="whitespace-pre-wrap font-mono text-xs text-slate-600">
              {this.state.errorInfo?.componentStack}
            </details>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}
