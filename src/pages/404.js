import React from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';

function NotFound() {
  return (
    <Layout title="Page Not Found" description="Page not found">
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '80vh',
        textAlign: 'center',
        padding: '2rem',
        background: 'linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%)',
      }}>
        <div style={{
          maxWidth: '800px',
          margin: '0 auto',
          padding: '2rem',
          backgroundColor: 'white',
          borderRadius: '12px',
          boxShadow: '0 8px 32px rgba(0,0,0,0.1)',
        }}>
          <div style={{
            fontSize: '5rem',
            color: '#2e8555',
            marginBottom: '1.5rem'
          }}>
            🤖
          </div>

          <h1 style={{
            fontSize: '2.5rem',
            color: '#2e8555',
            marginBottom: '1rem',
            fontWeight: 'bold'
          }}>
            Oops! Page Not Found
          </h1>

          <p style={{
            fontSize: '1.2rem',
            color: '#606060',
            marginBottom: '2rem',
            lineHeight: '1.6',
            maxWidth: '600px',
            margin: '0 auto 2rem'
          }}>
            We couldn't find what you were looking for. This page might have been moved,
            or the link you clicked might be broken. But don't worry, you're in the right place!
          </p>

          <div style={{
            marginBottom: '2rem',
            display: 'flex',
            justifyContent: 'center'
          }}>
            <img
              src={useBaseUrl('/img/logo.svg')}
              alt="Physical AI & Humanoid Robotics"
              style={{
                maxWidth: '180px',
                height: 'auto',
                filter: 'drop-shadow(0 4px 8px rgba(0,0,0,0.1))'
              }}
            />
          </div>

          <div style={{
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            gap: '1rem',
            marginTop: '2rem'
          }}>
            <a
              href={useBaseUrl('/docs/chapter-01')}
              style={{
                backgroundColor: '#2e8555',
                color: 'white',
                padding: '1rem 2rem',
                fontSize: '1.2rem',
                textDecoration: 'none',
                borderRadius: '8px',
                fontWeight: 'bold',
                boxShadow: '0 4px 12px rgba(46, 133, 85, 0.3)',
                transition: 'all 0.3s ease',
                display: 'inline-flex',
                alignItems: 'center',
                gap: '0.5rem'
              }}
              onMouseOver={(e) => {
                e.target.style.transform = 'translateY(-2px)';
                e.target.style.boxShadow = '0 6px 16px rgba(46, 133, 85, 0.4)';
              }}
              onMouseOut={(e) => {
                e.target.style.transform = 'translateY(0)';
                e.target.style.boxShadow = '0 4px 12px rgba(46, 133, 85, 0.3)';
              }}
            >
              <span>📚</span>
              Start Reading the Book
            </a>

            <p style={{
              fontSize: '1rem',
              color: '#888',
              margin: '1rem 0 0 0'
            }}>
              Or explore the navigation menu above to find what you're looking for.
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default NotFound;